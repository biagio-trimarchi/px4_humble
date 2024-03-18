#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// C++ Standard Libraries
#include <chrono>
#include <string>
#include <limits>

// Third Party Libraries

// Custom Libraries

// CLASSES
class T265ToOdometry : public rclcpp::Node {
	public:
		T265ToOdometry();
		~T265ToOdometry();

	private:
		// FUNCTIONS
		void T265Callback(const nav_msgs::msg::Odometry::SharedPtr msg_T265);
	
		// VARIABLES

		// Subscriptions
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_T265;

		// Publishers
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odometry;

		// TF
		std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};
