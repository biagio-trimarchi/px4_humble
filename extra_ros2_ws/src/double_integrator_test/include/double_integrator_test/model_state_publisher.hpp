#pragma once

// LIBRARIES
// ROS2 Libraries
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo_msgs/msg/model_state.hpp>
#include <gazebo_msgs/srv/get_model_state.hpp>

// C++ Standard Libraries
#include <string>
#include <chrono>

// Third Party Libraries

// CLASSES
class GazeboModelStatePublisher : public rclcpp::Node {
	public:
		// Constructors
		GazeboModelStatePublisher(std::string model_name);
		~GazeboModelStatePublisher();
	
	private:
		// FUNCTIONS
		void read_state_callback();
		void publish_state_callback(rclcpp::Client<gazebo_msgs::srv::GetModelState>::SharedFuture future);

		// INTERNAL VARIABLES
		std::string model_name;
		int timer_read_model_state_frequency_ms;
	
		// ROS2 VARIABLES
		// Timers
		rclcpp::TimerBase::SharedPtr timer_read_model_state;
		
		// Publishers
		rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_model_state;

		// Service clients
		rclcpp::Client<gazebo_msgs::srv::GetModelState>::SharedPtr client_model_state;
};
