#include <simulation_manager/gazebo_manager.hpp>

GazeboManager::GazeboManager() : Node("gazebo_manager") {
	// Declare parameters
	this->declare_parameter("topic_gazebo_drone_odometry", "/model/x500_vision_0/odometry");
	this->declare_parameter("topic_ros_drone_odometry", "/model/x500_vision_0/odometry");

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

	// Services
	service_logGPIS = this->create_service<log_gpis::srv::QueryEstimate>(
    "/barrier_function", 
    std::bind(&GazeboManager::logGPISCallback, this, 
              std::placeholders::_1, std::placeholders::_2)
  );

	// TF
	tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
	tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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
	
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GazeboManager>());
	rclcpp::shutdown();
	return 0;
}
