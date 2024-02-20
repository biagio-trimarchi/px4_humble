#include <camera_to_occupancy/camera_to_occupancy.hpp>

CameraToOccupancy::CameraToOccupancy() : Node("CameraToOccupancy") {
	// Set parameters
	this->declare_parameter("pointcloud_filter_min_distance", 0.5);
	this->declare_parameter("pointcloud_filter_max_distance", 2.5);
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
	sync_odom_cloud = std::make_shared<SynchronizerOdomCloud>(SyncPolicyOdomCloud(10), subscriber_pointcloud, subscriber_odometry);
	sync_odom_cloud->registerCallback(std::bind(&CameraToOccupancy::pointcloudCallback, this, std::placeholders::_1, std::placeholders::_2));

	// Publishers
	publisher_occupancy_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
	  "/out_occupancy_grid", qos_reliable_transient_local
	);

	publisher_debug_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(
	  "/out_debug_pointcloud", qos_reliable_transient_local
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

Eigen::Quaterniond CameraToOccupancy::yawRotationQuaternion(Eigen::Quaterniond q_orientation) {
	// Using the rotation conventtion of "Trajectory Generation and Control for Quadrotors" D.W.Mellinger
	// Can be found at https://core.ac.uk/download/pdf/76383736.pdf
	Eigen::Matrix3d rotation_matrix = q_orientation.toRotationMatrix();

	double yaw = std::atan2(rotation_matrix(0, 1), rotation_matrix(1, 1));

	Eigen::AngleAxisd yaw_axis_rotation(yaw, Eigen::Vector3d::UnitZ());

	Eigen::Quaterniond q_yaw_rotation(yaw_axis_rotation);
	return q_yaw_rotation;
}

void CameraToOccupancy::pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_pointcloud, const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odometry) {
	RCLCPP_INFO(this->get_logger(), "Reading odometry message...");
	// Read odometry
	drone_position.x() = msg_odometry->pose.pose.position.x;
	drone_position.y() = msg_odometry->pose.pose.position.y;
	drone_position.z() = msg_odometry->pose.pose.position.z;

	drone_orientation.x() = msg_odometry->pose.pose.orientation.x;
	drone_orientation.y() = msg_odometry->pose.pose.orientation.y;
	drone_orientation.z() = msg_odometry->pose.pose.orientation.z;
	drone_orientation.w() = msg_odometry->pose.pose.orientation.w;

	RCLCPP_INFO(this->get_logger(), "Preparing message...");
	// Read point cloud from topic
	pcl::fromROSMsg(*msg_pointcloud, current_pointcloud);

	// Filter points based on distance (remove points that are too near or too far)
	pcl::PassThrough<pcl::PointXYZ> box_filter;
	box_filter.setInputCloud(current_pointcloud.makeShared());
	box_filter.setFilterLimits(pointcloud_filter_min_distance, pointcloud_filter_max_distance);
	box_filter.setFilterFieldName("z");
	box_filter.filter(current_pointcloud);

	// Mark free space using a pointcloud (use 
	current_empty_space_pointcloud_2D.clear();
	RCLCPP_INFO(this->get_logger(), "Creating empty cloud...");
	for (float x = pointcloud_filter_min_distance; x <= pointcloud_filter_max_distance; x+=pointcloud_freespace_resolution) {
		for (float y = pointcloud_filter_min_distance; y <= pointcloud_filter_max_distance; y+=pointcloud_freespace_resolution) {
			// Point to check
			pcl::PointXYZ gridPoint;
			gridPoint.x = x;
			gridPoint.y = y;
			gridPoint.z = 0.0;

			current_empty_space_pointcloud_2D.points.push_back(gridPoint);
		}		
	}		

	// Align the pointcloud with the map
	Eigen::Matrix3d camera2image = Eigen::Matrix3d::Zero();
	camera2image(0, 2) =  1.0;
	camera2image(1, 0) = -1.0;
	camera2image(2, 1) = -1.0;
	Eigen::Affine3d odom2drone = Eigen::Translation3d(drone_position) * (drone_orientation * camera_orientation * camera2image);
	pcl::transformPointCloud(current_pointcloud, current_pointcloud, odom2drone);

	sensor_msgs::msg::PointCloud2 debug_pointcloud;
	pcl::toROSMsg(current_pointcloud, debug_pointcloud);
	debug_pointcloud.header.frame_id = this->get_parameter("frame_camera").as_string();
	debug_pointcloud.header.stamp = this->get_clock()->now();
	publisher_debug_pointcloud->publish(debug_pointcloud);

	Eigen::Quaterniond yaw_orientation(drone_orientation);
	Eigen::Affine3d odom2freespace_pointcloud = Eigen::Translation3d(drone_position) * yaw_orientation;
	pcl::transformPointCloud(current_empty_space_pointcloud_2D, current_empty_space_pointcloud_2D, odom2freespace_pointcloud);

	// Update occupancy map
	// Mark points as explored
	RCLCPP_INFO(this->get_logger(), "Mark explored");
	for (const pcl::PointXYZ& point : current_empty_space_pointcloud_2D.points) {
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

		// Mark point
		map_grid.data[index_x + index_y * map_grid.info.width] = 0;
	}

	// Mark points as occupied
	RCLCPP_INFO(this->get_logger(), "Mark blocked");
	for (const pcl::PointXYZ& point : current_pointcloud.points) {
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

		// Mark point
		map_grid.data[index_x + index_y * map_grid.info.width] = 127;
	}

	// Add known obstacles and arena walls
	// [TODO] Implement it

	map_grid.header.stamp = this->get_clock()->now();
	map_grid.header.frame_id = this->get_parameter("frame_map").as_string();
	
	publisher_occupancy_grid->publish(map_grid);
	RCLCPP_INFO(this->get_logger(), "Message sent!");
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraToOccupancy>());
	rclcpp::shutdown();
	return 0;
}
