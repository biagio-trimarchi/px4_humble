#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
		void debugCallback();
		void initializeOccupancyGrid();
		Eigen::Quaterniond yawRotationQuaternion(Eigen::Quaterniond orientation);
		void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud);
	
		// VARIABLES
		// Pointcloud
		pcl::PointCloud<pcl::PointXYZ> current_pointcloud;
		pcl::PointCloud<pcl::PointXYZ> current_empty_space_pointcloud_2D;
		float pointcloud_filter_min_distance;
		float pointcloud_filter_max_distance;
		float pointcloud_freespace_resolution;
		float pointcloud_camera_fov;

		// Occupancy grid
		nav_msgs::msg::OccupancyGrid map_grid;

		// Drone data
		Eigen::Vector3d drone_position;
		Eigen::Quaterniond drone_orientation;
		Eigen::Quaterniond camera_orientation;

		// Subscriptions
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_pointcloud;

		// Publishers
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_occupancy_grid;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_debug_pointcloud;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_debug_empty_pointcloud;
};
