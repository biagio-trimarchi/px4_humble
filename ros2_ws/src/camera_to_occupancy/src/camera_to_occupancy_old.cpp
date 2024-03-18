#include <camera_to_occupancy/camera_to_occupancy.hpp>

CameraToOccupancy::CameraToOccupancy() : Node("CameraToOccupancy") {
	// Set parameters
	this->declare_parameter("pointcloud_filter_min_distance", 1.5);
	this->declare_parameter("pointcloud_filter_max_distance", 3.5);
	this->declare_parameter("pointcloud_freespace_resolution", 0.1);
	this->declare_parameter("camera_orientation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
	this->declare_parameter("map_width", 10.0);
	this->declare_parameter("map_height", 12.0);
	this->declare_parameter("occupancy_grid_resolution", 0.2);
	this->declare_parameter("frame_map", "camera_link");
	this->declare_parameter("frame_camera", "camera_link");

	// Temp parameters
	// Get parameters
	this->get_parameter("pointcloud_filter_min_distance", pointcloud_filter_min_distance);
	this->get_parameter("pointcloud_filter_max_distance", pointcloud_filter_max_distance);
	this->get_parameter("pointcloud_freespace_resolution", pointcloud_freespace_resolution);
	rclcpp::Parameter param_camera_orientation = this->get_parameter("camera_orientation");
	camera_orientation = Eigen::Quaterniond(
	  (param_camera_orientation.as_double_array())[0], // w
	  (param_camera_orientation.as_double_array())[1], // x 
	  (param_camera_orientation.as_double_array())[2], // y
	  (param_camera_orientation.as_double_array())[3]  // z 
	);

	// Initialization
	current_pointcloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	drone_position = Eigen::Vector3d(0.0, 0.0, 0.0);
	drone_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
	initializeOccupancyGrid();
	
	// QoS
	rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos_sensor_data = rclcpp::QoS(
	  rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
	  qos_profile_sensor_data
	);

	rclcpp::QoS qos_reliable_transient_local(1);
	qos_reliable_transient_local.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	qos_reliable_transient_local.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
	qos_reliable_transient_local.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
	
	// Subscriptions
	subscriber_pointcloud.subscribe(this, "/in_cloud", rmw_qos_profile_sensor_data);
	subscriber_odometry.subscribe(this, "/in_odometry", rmw_qos_profile_sensor_data);
	sync_odom_cloud = std::make_shared<SynchronizerOdomCloud>(SyncPolicyOdomCloud(100), subscriber_pointcloud, subscriber_odometry);
	sync_odom_cloud->registerCallback(std::bind(&CameraToOccupancy::pointcloudCallback, this, std::placeholders::_1, std::placeholders::_2));

	// Publishers
	publisher_occupancy_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
	  "/out_occupancy_grid", qos_reliable_transient_local
	);

	publisher_debug_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
	  "/out_debug_pointcloud", qos_reliable_transient_local
	);

	publisher_empty_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
	  "/out_empty_pointcloud", qos_reliable_transient_local
	);
}

CameraToOccupancy::~CameraToOccupancy() {}

void CameraToOccupancy::initializeOccupancyGrid() {
	map_grid.info.width = int(
	  std::floor(
	    this->get_parameter("map_width").as_double() / this->get_parameter("occupancy_grid_resolution").as_double()
	  )
	);
	map_grid.info.height = int(
	  std::floor(
	    this->get_parameter("map_height").as_double() / this->get_parameter("occupancy_grid_resolution").as_double()
	  )
	);
	map_grid.info.resolution = this->get_parameter("occupancy_grid_resolution").as_double();
	map_grid.data = std::vector<signed char>(map_grid.info.width * map_grid.info.height, -1);

	// Add known obstacles and walls
	// [TODO]
}

double CameraToOccupancy::yawRotationQuaternion(Eigen::Quaterniond q_orientation) {
	// Using the rotation conventtion of "Trajectory Generation and Control for Quadrotors" D.W.Mellinger
	// Can be found at https://core.ac.uk/download/pdf/76383736.pdf
	Eigen::Matrix3d rotation_matrix = q_orientation.toRotationMatrix();

	return std::atan2(-rotation_matrix(0, 1), rotation_matrix(1, 1));
}

void CameraToOccupancy::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_pointcloud, const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odometry) {
	RCLCPP_INFO(this->get_logger(), "Starting callback!");
	
	// Read odometry
	drone_position.x() = msg_odometry->pose.pose.position.x;
	drone_position.y() = msg_odometry->pose.pose.position.y;
	drone_position.z() = msg_odometry->pose.pose.position.z;

	drone_orientation.x() = msg_odometry->pose.pose.orientation.x;
	drone_orientation.y() = msg_odometry->pose.pose.orientation.y;
	drone_orientation.z() = msg_odometry->pose.pose.orientation.z;
	drone_orientation.w() = msg_odometry->pose.pose.orientation.w;

	// Read point cloud from topic
	pcl::fromROSMsg(*msg_pointcloud, *current_pointcloud);
	if (current_pointcloud->size() < 100) {
		RCLCPP_INFO(this->get_logger(), "Empty cloud, returning");
		return;
	}

	// Filter 1 
	// Voxelize pointcloud to speed up following filterings
	pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
	voxel_filter.setInputCloud(current_pointcloud);
	voxel_filter.setLeafSize(0.05f, 0.05f, 0.05f);
	voxel_filter.filter(*current_pointcloud);
	if (current_pointcloud->size() < 100){
		RCLCPP_INFO(this->get_logger(), "Empty cloud, returning");
		return;
	}

	// Filter 2
	// Remove far away points from the scene
	Eigen::Matrix3f filter_matrix_A = Eigen::Matrix3f::Identity();
	Eigen::Vector3f filter_vector_c = Eigen::Vector3f::Zero();
	float filter_scalar = - pointcloud_filter_max_distance * pointcloud_filter_max_distance;
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr distance_condition(new pcl::ConditionAnd<pcl::PointXYZ>);
	distance_condition->addComparison(
			pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::ConstPtr (
				new pcl::TfQuadraticXYZComparison<pcl::PointXYZ> (
					pcl::ComparisonOps::LT, filter_matrix_A, filter_vector_c, filter_scalar
					)
				)
			);
	pcl::ConditionalRemoval<pcl::PointXYZ> distance_filter;
	distance_filter.setCondition(distance_condition);
	distance_filter.setInputCloud(current_pointcloud);
	distance_filter.setKeepOrganized(true);
	distance_filter.filter(*current_pointcloud);
	if (current_pointcloud->size() < 100){
		RCLCPP_INFO(this->get_logger(), "Empty cloud, returning");
		return;
	}

	// Align the pointcloud with the map
	Eigen::Matrix3d camera2image = Eigen::Matrix3d::Zero();
	camera2image(0, 2) =  1.0;
	camera2image(1, 0) = -1.0;
	camera2image(2, 1) = -1.0;
	Eigen::Affine3d odom2drone = Eigen::Affine3d::Identity();
	odom2drone.translation() = drone_position;
	odom2drone.linear() = (drone_orientation * camera_orientation).toRotationMatrix() * camera2image;
	pcl::transformPointCloud(*current_pointcloud, *current_pointcloud, odom2drone);

	// Filtering 3
	// Remove ceiling
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr ceiling_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
	ceiling_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 2.0)));
	pcl::ConditionalRemoval<pcl::PointXYZ> ceiling_remover;
	ceiling_remover.setCondition(ceiling_condition);
	ceiling_remover.setInputCloud(current_pointcloud);
	ceiling_remover.setKeepOrganized(true);
	ceiling_remover.filter(*current_pointcloud);
	if (current_pointcloud->size() < 100){
		RCLCPP_INFO(this->get_logger(), "Empty cloud, returning");
		return;
	}

	// Filtering 4
	// Remove ground
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr ground_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
	ground_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, 0.2)));
	pcl::ConditionalRemoval<pcl::PointXYZ> ground_remover;
	ground_remover.setCondition(ground_condition);
	ground_remover.setInputCloud(current_pointcloud);
	ground_remover.setKeepOrganized(true);
	ground_remover.filter(*current_pointcloud);
	if (current_pointcloud->size() < 100){
		RCLCPP_INFO(this->get_logger(), "Empty cloud, returning");
		return;
	}

	// Filtering 4 
	// Remove ground
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
	// pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
	// pcl::SACSegmentation<pcl::PointXYZ> plane_segmentation;
	// plane_segmentation.setOptimizeCoefficients(false);
	// plane_segmentation.setModelType(pcl::SACMODEL_PLANE);
	// plane_segmentation.setMethodType(pcl::SAC_RANSAC);
	// plane_segmentation.setMaxIterations(1000);
	// plane_segmentation.setDistanceThreshold(0.20);
// 
	// plane_segmentation.setInputCloud(current_pointcloud);
	// plane_segmentation.segment(*plane_inliers, *plane_coefficients);
// 
	// pcl::ExtractIndices<pcl::PointXYZ> extract_plane;
	// extract_plane.setInputCloud(current_pointcloud);
	// extract_plane.setIndices(plane_inliers);
	// extract_plane.setNegative(true);
	// extract_plane.filter(*current_pointcloud);

	// Extract Objects
	// RCLCPP_INFO(this->get_logger(), "Pointcloud size: %ld", current_pointcloud->size());
	// pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// tree->setInputCloud(current_pointcloud);
	// std::vector<pcl::PointIndices> cluster_indices;
	// pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidean_clusters;
	// euclidean_clusters.setClusterTolerance(0.10);
	// euclidean_clusters.setMinClusterSize(50);
	// euclidean_clusters.setMaxClusterSize(25000);
	// euclidean_clusters.setSearchMethod(tree);
	// euclidean_clusters.setInputCloud(current_pointcloud);
	// euclidean_clusters.extract(cluster_indices);

	// Prepare projection
	// pcl::ModelCoefficients::Ptr projection_coefficients(new pcl::ModelCoefficients());
	// projection_coefficients->values.resize(4);
	// projection_coefficients->values[0] = projection_coefficients->values[1] = 0.0;
	// projection_coefficients->values[2] = 1.0;
	// projection_coefficients->values[3] = 0.0;
	// pcl::ProjectInliers<pcl::PointXYZ> plane_projection;
	// plane_projection.setModelType(pcl::SACMODEL_PLANE);
	// plane_projection.setModelCoefficients(projection_coefficients);

	double yaw_orientation = yawRotationQuaternion(drone_orientation);
	pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Fill the sensed section with blank
	for (float r = pointcloud_filter_min_distance; r <= pointcloud_filter_max_distance; r+=pointcloud_freespace_resolution) {
		for (float theta = -M_PI/6.0; theta <= M_PI/6.0; theta+=0.1) {
			// Point to check
			float x, y;
			int index_x, index_y;
			x = drone_position.x() + r * std::cos(theta + yaw_orientation);
			y = drone_position.y() + r * std::sin(theta + yaw_orientation);

			pcl::PointXYZ point;
			point.x = x;
			point.y = y;
			point.z = 0.0;
			empty_pointcloud->push_back(point);

			index_x = int(std::floor(x / map_grid.info.resolution));
			if ((unsigned int) index_x > map_grid.info.width-1 || index_x < 1) {
				// If the point is out of bounds, ignore point
				continue;
			}

			index_y = int(std::floor(y / map_grid.info.resolution));
			if ((unsigned int) index_y > map_grid.info.height-1 || index_y < 1) {
				// If the point is out of bounds, ignore point
				continue;
			}

			// Mark point as explored
			if (map_grid.data[index_x + index_y * map_grid.info.width] != 127)
				map_grid.data[index_x + index_y * map_grid.info.width] = 0;
		}		
	}		

	// Project each detected cluster on to the plane and fill "imaginary sensor"
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZ>);
	// for (const auto& cluster : cluster_indices) {
		// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		// for (const auto& idx : cluster.indices) {
			// cloud_cluster->push_back((*current_pointcloud)[idx]);
		// }
// 
		// cloud_cluster->width = cloud_cluster->size();
		// cloud_cluster->height = 1;
		// cloud_cluster->is_dense = true;
		// plane_projection.setInputCloud(cloud_cluster);
		// plane_projection.filter(*cloud_cluster);
// 
		// *cloud_sum += *cloud_cluster;
		// cloud_sum->width = cloud_sum->size();
		// cloud_sum->height = 1;
		// cloud_sum->is_dense = true;
// 
		// for (const pcl::PointXYZ& point : cloud_cluster->points) {
			// int index_x, index_y;
// 
			// // Convert coordinate to grid point
			// // x coordinate
			// index_x = int(std::floor(point.x / map_grid.info.resolution));
			// if ((unsigned int) index_x > map_grid.info.width-1 || index_x < 1) {
				// // If the point is out of bounds, ignore point
				// continue;
			// }
// 
			// // y coordinate	
			// index_y = int(std::floor(point.y / map_grid.info.resolution));
			// if ((unsigned int) index_y > map_grid.info.height-1 || index_y < 1) {
				// // If the point is out of bounds, ignore point
				// continue;
			// }
// 
			// // Mark point as occupied
			// map_grid.data[index_x + index_y * map_grid.info.width] = 127;
		// }
	// }

	std::vector<int> modified_index;
	std::vector<int> counters;
	for (const pcl::PointXYZ& point : current_pointcloud->points) {
		int index_x, index_y;

		// Convert coordinate to grid point
		// x coordinate
		index_x = int(std::floor(point.x / map_grid.info.resolution));
		if ((unsigned int) index_x > map_grid.info.width-1 || index_x < 1) {
			// If the point is out of bounds, ignore point
			continue;
		}

		// y coordinate	
		index_y = int(std::floor(point.y / map_grid.info.resolution));
		if ((unsigned int) index_y > map_grid.info.height-1 || index_y < 1) {
			// If the point is out of bounds, ignore point
			continue;
		}

		// Mark point as occupied
		int grid_coordinate = index_x + index_y * map_grid.info.width;
		std::vector<int>::iterator it = std::find(modified_index.begin(), modified_index.end(), grid_coordinate);
		if (it == modified_index.end()) {
			modified_index.push_back(grid_coordinate);
			counters.push_back(1);
		} else {
			int index = it - modified_index.begin();
			counters[index]++;
		}
	}

	for (unsigned int index = 0; index < counters.size(); index++) {
		if (counters[index] > 50) {
			map_grid.data[modified_index[index]] = 127;
		}
	}

	modified_index.clear();
	counters.clear();

	// Debug
	sensor_msgs::msg::PointCloud2 debug_pointcloud;
	pcl::toROSMsg(*current_pointcloud, debug_pointcloud);
	debug_pointcloud.header.frame_id = this->get_parameter("frame_camera").as_string();
	debug_pointcloud.header.stamp = this->get_clock()->now();
	publisher_debug_pointcloud->publish(debug_pointcloud);

	// Debug
	pcl::toROSMsg(*empty_pointcloud, debug_pointcloud);
	debug_pointcloud.header.frame_id = this->get_parameter("frame_camera").as_string();
	debug_pointcloud.header.stamp = this->get_clock()->now();
	publisher_empty_pointcloud->publish(debug_pointcloud);

	// Mark points as occupied

	// Add known obstacles and arena walls
	// [TODO] Implement it

	map_grid.header.stamp = this->get_clock()->now();
	map_grid.header.frame_id = this->get_parameter("frame_map").as_string();
	
	publisher_occupancy_grid->publish(map_grid);
	RCLCPP_INFO(this->get_logger(), "Finished callback!");
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraToOccupancy>());
	rclcpp::shutdown();
	return 0;
}
