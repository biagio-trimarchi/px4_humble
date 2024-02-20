#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <vicon_receiver/msg/position.hpp>
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
class ViconToOdometry : public rclcpp::Node {
	public:
		ViconToOdometry();
		~ViconToOdometry();

	private:
		// FUNCTIONS
		void viconCallback(const vicon_receiver::msg::Position::SharedPtr msg_vicon);
	
		// VARIABLES

		// Subscriptions
		rclcpp::Subscription<vicon_receiver::msg::Position>::SharedPtr subscriber_vicon;

		// Publishers
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odometry;

		// TF
		std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};
