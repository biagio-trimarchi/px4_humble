#include <vicon_manager/vicon_to_odometry.hpp>

ViconToOdometry::ViconToOdometry() : Node("ViconToOdometry") {
	// Set parameters
	this->declare_parameter("tf_parent", "map");
	this->declare_parameter("tf_child", "odom");

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
	subscriber_vicon = this->create_subscription<vicon_receiver::msg::Position>(
	  "/topic_vicon", qos_sensor_data,
	  std::bind(&ViconToOdometry::viconCallback, this, std::placeholders::_1)
	);

	// Publishers
	publisher_odometry = this->create_publisher<nav_msgs::msg::Odometry>(
	  "/topic_odometry", qos_sensor_data
	);
	
	// TF
	tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

ViconToOdometry::~ViconToOdometry() {}

void ViconToOdometry::viconCallback(const vicon_receiver::msg::Position::SharedPtr msg_vicon) {
	nav_msgs::msg::Odometry msg_odometry;
	geometry_msgs::msg::TransformStamped transform;

	// Odometry
	// Fill header
	auto stamp = this->get_clock()->now();
	msg_odometry.header.stamp = stamp;
	msg_odometry.header.frame_id = this->get_parameter("tf_parent").as_string();

	// Fill message
	// Position and orientation
	msg_odometry.pose.pose.position.x = msg_vicon->x_trans * 0.001;
	msg_odometry.pose.pose.position.y = msg_vicon->y_trans * 0.001;
	msg_odometry.pose.pose.position.z = msg_vicon->z_trans * 0.001;

	msg_odometry.pose.pose.orientation.x = msg_vicon->x_rot;
	msg_odometry.pose.pose.orientation.y = msg_vicon->y_rot;
	msg_odometry.pose.pose.orientation.z = msg_vicon->z_rot;
	msg_odometry.pose.pose.orientation.w = msg_vicon->w;

	// Twist (Nan - Unknown)
	// TODO: Implement a filter to estimate velocities from Vicon readings
	msg_odometry.twist.twist.linear.x = std::numeric_limits<float>::quiet_NaN();
	msg_odometry.twist.twist.linear.y = std::numeric_limits<float>::quiet_NaN();
	msg_odometry.twist.twist.linear.z = std::numeric_limits<float>::quiet_NaN();

	msg_odometry.twist.twist.angular.x = std::numeric_limits<float>::quiet_NaN();
	msg_odometry.twist.twist.angular.y = std::numeric_limits<float>::quiet_NaN();
	msg_odometry.twist.twist.angular.z = std::numeric_limits<float>::quiet_NaN();

	publisher_odometry->publish(msg_odometry);

	// tf
	transform.header.stamp = stamp;
	transform.header.frame_id = this->get_parameter("tf_parent").as_string();
	transform.child_frame_id = this->get_parameter("tf_child").as_string();
	
	transform.transform.translation.x = msg_vicon->x_trans * 0.001;
	transform.transform.translation.y = msg_vicon->y_trans * 0.001;
	transform.transform.translation.z = msg_vicon->z_trans * 0.001;

	transform.transform.rotation.x = msg_vicon->x_rot;
	transform.transform.rotation.y = msg_vicon->y_rot;
	transform.transform.rotation.z = msg_vicon->z_rot;
	transform.transform.rotation.w = msg_vicon->w;

	tf_broadcaster->sendTransform(transform);
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ViconToOdometry>());
	rclcpp::shutdown();
	return 0;
}
