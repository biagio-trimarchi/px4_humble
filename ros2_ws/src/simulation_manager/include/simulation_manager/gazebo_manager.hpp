#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_ros_com/frame_transforms.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// C++ Standard Libraries
#include <string>
#include <chrono>

// Third Party Libraries
#include <Eigen/Eigen>

// Custom Libraries
#include <simulation_manager/world_manager.hpp>
#include <log_gpis/logGPIS.hpp>
#include <log_gpis/srv/query_estimate.hpp>

// CLASSES
class GazeboManager : public rclcpp::Node {
	public:
		GazeboManager();
		~GazeboManager();

	private:
		// FUNCTIONS
		void initializeWorld();
		void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg);
		void publishTFs();
		void logGPISCallback(const std::shared_ptr<log_gpis::srv::QueryEstimate::Request> request,
                         const std::shared_ptr<log_gpis::srv::QueryEstimate::Response> response);
		void visualizationCallback();
		void scanRegion();
	
		// VARIABLES
		std::string topic_gazebo_drone_odometry;
		std::string topic_ros_drone_odometry;
		std::string topic_visualization;
		std::string tf_world_name;
		std::string tf_odometry_name;
		std::string tf_drone_name;
		std::string world_name;
		bool load_world;
		bool save_world;

		LogGPIS log_gpis;
		double gp_lambda_whittle;
		double gp_resolution;
		double gp_error_variance;

		visualization_msgs::msg::Marker debug_message;

		// Callback groups
		rclcpp::CallbackGroup::SharedPtr callback_group_visualization;
		
		// Timers
		rclcpp::TimerBase::SharedPtr timer_visualization;

		// Subscriptions
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_drone_odometry;

		// Publishers
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_drone_odometry;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_visualization;
		rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_px4_odometry;

		// Services
		rclcpp::Service<log_gpis::srv::QueryEstimate>::SharedPtr service_logGPIS;

		// TF
		std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster;
		std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};
