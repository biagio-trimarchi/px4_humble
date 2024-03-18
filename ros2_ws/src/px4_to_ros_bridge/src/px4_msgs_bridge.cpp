/*
 * @file px4_msgs_bridge.cpp
 *
 * @brief Node that bridge PX4 messages to ROS 2 in simulation
 *
 * @date 2023-10-23
 *
 * @version 1.0.0
 *
 */
// // LIBRARIES

// ROS 2
#include <rclcpp/rclcpp.hpp>

#include <px4_ros_com/frame_transforms.h>
// ROS 2 messages
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <nav_msgs/msg/odometry.hpp>

// C++ Standard Libraries

// Third Party Libraries

// // NAMESPACES
using std::placeholders::_1;

// // CLASS DEFINITIONS
class Px4Bridge : public rclcpp::Node {
	public:
		Px4Bridge() : Node("px4_msgs_bridge") {
			// // PARAMETERS
			// Declare parameters	
			
			// Get parameters

			// QOS
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
			
			// // SUBSCRIBERS
			subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
				"/fmu/out/vehicle_odometry", qos, std::bind(&Px4Bridge::subscription_callback, this, _1)
			);

			// // PUBLISHERS
			publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/simulation_vehicle_odometry", qos);
			
			// TF2
		}
	
	private:
		// // FUNCTIONS
		void subscription_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
			// Convert message
 			drone_translation = px4_ros_com::frame_transforms::ned_to_enu_local_frame(Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]));
      			drone_velocity = px4_ros_com::frame_transforms::ned_to_enu_local_frame(Eigen::Vector3d(msg->velocity[0], msg->velocity[1], msg->velocity[2]));
      			drone_angular_velocity = px4_ros_com::frame_transforms::ned_to_enu_local_frame(Eigen::Vector3d(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]));
      			drone_orientation = px4_ros_com::frame_transforms::px4_to_ros_orientation(Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3])); 

			// Publish message
			nav_msgs::msg::Odometry new_msg;
			
			new_msg.header.stamp = this->get_clock()->now();
			new_msg.header.frame_id = "odom";

			new_msg.child_frame_id = "drone";

			new_msg.pose.pose.position.x = drone_translation(0);
			new_msg.pose.pose.position.y = drone_translation(1);
			new_msg.pose.pose.position.z = drone_translation(2);

			new_msg.pose.pose.orientation.w = drone_orientation.w();
			new_msg.pose.pose.orientation.x = drone_orientation.x();
			new_msg.pose.pose.orientation.y = drone_orientation.y();
			new_msg.pose.pose.orientation.z = drone_orientation.z();

			new_msg.twist.twist.linear.x = drone_velocity(0);
			new_msg.twist.twist.linear.y = drone_velocity(1);
			new_msg.twist.twist.linear.z = drone_velocity(2);

			new_msg.twist.twist.angular.x = drone_angular_velocity(0);
			new_msg.twist.twist.angular.y = drone_angular_velocity(1);
			new_msg.twist.twist.angular.z = drone_angular_velocity(2);

			publisher_->publish(new_msg);
		}
		
		// // VARIABLES
		// Other Variables
		Eigen::Vector3d drone_translation = Eigen::Vector3d::Zero();
		Eigen::Vector3d drone_velocity = Eigen::Vector3d::Zero();
		Eigen::Vector3d drone_angular_velocity = Eigen::Vector3d::Zero();
		Eigen::Quaterniond drone_orientation = Eigen::Quaterniond::Identity();

		// Subscribers
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
		
		// Publishers
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
		// TF2
};

int main(int argc, char **argv) {
	// // ROS 2 SETUP
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Px4Bridge>());
	rclcpp::shutdown();
	return 0;
}
