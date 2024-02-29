#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// C++ Standard Libraries
#include <chrono>
#include <cmath>
#include <vector>

// Third Party Libraries
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>

// Custom Libraries

// CLASSES
class CameraToOccupancy : public rclcpp::Node {
	public:
		CameraToOccupancy();
		~CameraToOccupancy();

	private:
		// FUNCTIONS
		void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);
	
		// VARIABLES
		// Pointcloud
		// Occupancy grid
		// Drone data
		// Subscriptions
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_pointcloud;
		// Publishers
};
