#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/conditional_removal.h>

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
		double yawRotationQuaternion(Eigen::Quaterniond orientation);
		void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg_pointcloud, const nav_msgs::msg::Odometry::ConstSharedPtr& msg_odometry);
	
		// VARIABLES
		// Pointcloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr current_pointcloud;
		float pointcloud_filter_min_distance;
		float pointcloud_filter_max_distance;
		float pointcloud_freespace_resolution;

		// Occupancy grid
		nav_msgs::msg::OccupancyGrid map_grid;

		// Drone data
		Eigen::Vector3d drone_position;
		Eigen::Quaterniond drone_orientation;
		Eigen::Quaterniond camera_orientation;

		// Subscriptions
		message_filters::Subscriber<sensor_msgs::msg::PointCloud2> subscriber_pointcloud;
		message_filters::Subscriber<nav_msgs::msg::Odometry> subscriber_odometry;

		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry> SyncPolicyOdomCloud;
		typedef message_filters::Synchronizer<SyncPolicyOdomCloud> SynchronizerOdomCloud;
		std::shared_ptr<SynchronizerOdomCloud> sync_odom_cloud;

		// Publishers
		rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_occupancy_grid;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_debug_pointcloud;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_empty_pointcloud;
};
