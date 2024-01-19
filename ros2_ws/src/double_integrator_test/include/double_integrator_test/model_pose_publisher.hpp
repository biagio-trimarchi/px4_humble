#pragma once

// LIBRARIES
#pragma once

// LIBRARIES
// ROS2 Libraries
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_state.hpp>
#include <gazebo_msgs/srv/get_model_state.hpp>

// C++ Standard Libraries
#include <string>

// Third Party Libraries

// CLASSES
class GazeboModelPosePublisher {
	public:
		GazeboModelPosePublisher(std::string Name);
		~GazeboModelPosePublisher();
	
	private:
		// VARIABLES
		std::string name;
	
		// Publishers
		rclcpp::Publisher<gazebo_msgs::msg::ModelState>::SharedPtr publisher_model_state;

		// Service clients
		rclcpp::Client<gazebo_msgs::srv::GetModelState>::SharedPtr client_model_state;
}
