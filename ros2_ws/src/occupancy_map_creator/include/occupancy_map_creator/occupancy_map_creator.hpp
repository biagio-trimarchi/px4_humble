#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

// C++ Standard Libraries
#include <chrono>
#include <vector>

// Third Party Libraries

// Custom Libraries

// CLASSES
class LDCMapCreator : public rclcpp::Node {
	public:
		LDCMapCreator();
		~LDCMapCreator();

	private:
		// FUNCTIONS
		void timerCallback();
		void createMap();
		void mapERF2024();
		void addRectangle(double lower_left_corner_x,
		                  double lower_left_corner_y, 
		                  double lenght_x, double lenght_y);
	
		// VARIABLES
		nav_msgs::msg::OccupancyGrid map;

		// Timers
		rclcpp::TimerBase::SharedPtr timer_map;

		// Publishers
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_map;
};
