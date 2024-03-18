#include <vicon_manager/T265_to_odometry.hpp>

T265ToOdometry::T265ToOdometry() : Node("T265ToOdometry") {
	// Set parameters
	this->declare_parameter("offset_x", 0.0);
	this->declare_parameter("offset_y", 0.0);
	this->declare_parameter("offset_z", 0.0);
	this->declare_parameter("tf_parent", "map");
	this->declare_parameter("tf_child", "camera_link");

	// Get paramters
	// this->get_parameter("parameter", parameter);
	
	// Timers
	// QoS
	rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos_sensor_data = rclcpp::QoS(
	  rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
	  qos_profile_sensor_data
	);

	// Subscriptions
	subscriber_T265 = this->create_subscription<nav_msgs::msg::Odometry>(
	  "/camera/pose/sample", qos_sensor_data,
	  std::bind(&T265ToOdometry::T265Callback, this, std::placeholders::_1)
	);

	// Publishers
	publisher_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
	  "/topic_odometry", qos_sensor_data
	);
	
	// TF
	tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

T265ToOdometry::~T265ToOdometry() {}

void T265ToOdometry::T265Callback(const nav_msgs::msg::Odometry::SharedPtr msg_T265) {
	nav_msgs::msg::Odometry msg_odometry;
	geometry_msgs::msg::TransformStamped transform;
	msg_odometry = *msg_T265;

	// Odometry
	// Fill header
	auto stamp = this->get_clock()->now();
	msg_odometry.header.stamp = stamp;
	msg_odometry.header.frame_id = this->get_parameter("tf_parent").as_string();

	// Fill message
	// Position offset
	msg_odometry.pose.pose.position.x += this->get_parameter("offset_x").as_double();
	msg_odometry.pose.pose.position.y += this->get_parameter("offset_y").as_double();
	msg_odometry.pose.pose.position.z += this->get_parameter("offset_z").as_double();

	publisher_odometry->publish(msg_odometry);

	// tf
	transform.header.stamp = stamp;
	transform.header.frame_id = this->get_parameter("tf_parent").as_string();
	transform.child_frame_id = this->get_parameter("tf_child").as_string();
	
	transform.transform.translation.x = msg_odometry.pose.pose.position.x;
	transform.transform.translation.y = msg_odometry.pose.pose.position.y;
	transform.transform.translation.z = msg_odometry.pose.pose.position.z;

	transform.transform.rotation.x = msg_odometry.pose.pose.orientation.x;
	transform.transform.rotation.y = msg_odometry.pose.pose.orientation.y;
	transform.transform.rotation.z = msg_odometry.pose.pose.orientation.z;
	transform.transform.rotation.w = msg_odometry.pose.pose.orientation.w;

	tf_broadcaster->sendTransform(transform);
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<T265ToOdometry>());
	rclcpp::shutdown();
	return 0;
}
