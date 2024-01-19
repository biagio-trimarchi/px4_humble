#include <simulation_manager/gazebo_manager.hpp>

GazeboManager::GazeboManager() : Node("gazebo_manager") {
	// Declare parameters
	this->declare_parameter("topic_gazebo_drone_odometry", "/model/x500_vision_0/odometry");
	this->declare_parameter("topic_ros_drone_odometry", "/model/x500_vision_0/odometry");
	this->declare_parameter("topic_visualization", "/log_gpis_visualization");

	this->declare_parameter("tf_world_name", "map");
	this->declare_parameter("tf_odometry_name", "odometry");
	this->declare_parameter("tf_drone_name", "drone");
	this->declare_parameter("world_name", "lgpis_test_1");

	this->declare_parameter("gp_lambda_whittle", 20.0);
	this->declare_parameter("gp_resolution", 0.1);
	this->declare_parameter("gp_error_variance", 0.01);

	this->declare_parameter("load_world", true);
	this->declare_parameter("save_world", false);
	
	// Get parameters
	this->get_parameter("topic_gazebo_drone_odometry", topic_gazebo_drone_odometry);
	this->get_parameter("topic_ros_drone_odometry", topic_ros_drone_odometry);
	this->get_parameter("topic_visualization", topic_visualization);
	this->get_parameter("tf_world_name", tf_world_name);
	this->get_parameter("tf_odometry_name", tf_odometry_name);
	this->get_parameter("tf_drone_name", tf_drone_name);
	this->get_parameter("world_name", world_name);
	this->get_parameter("gp_lambda_whittle", gp_lambda_whittle);
	this->get_parameter("gp_resolution", gp_resolution);
	this->get_parameter("gp_error_variance", gp_error_variance);

	this->get_parameter("load_world", load_world);
	this->get_parameter("save_world", save_world);

	// Timers
	timer_visualization = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&GazeboManager::visualizationCallback, this)
  );

	// QoS
	rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos_sensor_data = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
    qos_profile_sensor_data
  );
	
	// Subscriptions
	subscriber_drone_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
    topic_gazebo_drone_odometry, qos_sensor_data,
		std::bind(&GazeboManager::odometryCallback, this, std::placeholders::_1)
  );
	
	// Publishers
	publisher_drone_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
    topic_ros_drone_odometry, qos_sensor_data	
  );

	publisher_visualization = this->create_publisher<visualization_msgs::msg::Marker>(
    topic_visualization, qos_sensor_data	
  );

	// Services
	service_logGPIS = this->create_service<log_gpis::srv::QueryEstimate>(
    "/barrier_function", 
    std::bind(&GazeboManager::logGPISCallback, this, 
              std::placeholders::_1, std::placeholders::_2)
  );

	// TF
	tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
	tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

	debug_message = visualization_msgs::msg::Marker();
	initializeWorld();
}

GazeboManager::~GazeboManager() {}

void GazeboManager::initializeWorld() {
	log_gpis = LogGPIS(gp_lambda_whittle, gp_resolution, gp_error_variance);
	std::string path_data_directory = ament_index_cpp::get_package_share_directory("simulation_manager") + "/data/";
	if (load_world) {
		loadWorld(world_name, path_data_directory, log_gpis);
		return;
	} 

	buildWorld(world_name, log_gpis);

	if (save_world) {
		saveWorld(world_name, path_data_directory, log_gpis);
	}
}

void GazeboManager::odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
  nav_msgs::msg::Odometry odometry_msg = *(msg.get());
	odometry_msg.header.stamp = this->now();
}

void GazeboManager::logGPISCallback(const std::shared_ptr<log_gpis::srv::QueryEstimate::Request> request,
                                    const std::shared_ptr<log_gpis::srv::QueryEstimate::Response> response) {
}

void GazeboManager::visualizationCallback() {
	RCLCPP_INFO(this->get_logger(), "Preparing message...");

	debug_message.header.frame_id = "map";
	debug_message.header.stamp = this->get_clock()->now();
	debug_message.type = visualization_msgs::msg::Marker::POINTS;
	debug_message.action = visualization_msgs::msg::Marker::ADD;
	debug_message.points.clear();
	scanRegion();
	debug_message.pose.position.x = 0.0;
	debug_message.pose.position.y = 0.0;
	debug_message.pose.position.z = 0.0;
	debug_message.pose.orientation.w = 1.0;
	debug_message.pose.orientation.x = 0.0;
	debug_message.pose.orientation.y = 0.0;
	debug_message.pose.orientation.z = 0.0;
	debug_message.scale.x = 0.1;
	debug_message.scale.y = 0.1;
	debug_message.scale.z = 0.1;
	debug_message.color.r = 0.0;
	debug_message.color.g = 1.0;
	debug_message.color.b = 0.0;
	debug_message.color.a = 1.0;

	publisher_visualization->publish(debug_message);
	RCLCPP_INFO(this->get_logger(), "Message sent!");
}

void GazeboManager::scanRegion() {
	double scan_resolution = 0.2;
	for (double x = -5.0; x < 5.1; x+=scan_resolution){
		for (double y = -5.0; y < 5.1; y+=scan_resolution){
			for (double z = -5.0; z < 6.1; z+=scan_resolution){
				Eigen::Vector3d scan_point(x, y, z);
				if (log_gpis.evaluate(scan_point) < 0.1) {
					geometry_msgs::msg::Point point;
					point.x = x;
					point.y = y;
					point.z = z;
					debug_message.points.push_back(point);
				}
			}
		}
	}

}
	
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GazeboManager>());
	rclcpp::shutdown();
	return 0;
}