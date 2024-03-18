#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// C++ Standard Libraries
#include <chrono>
#include <math.h>
#include <limits>

// Third Party Libraries
#include <Eigen/Eigen>

// Custom Libraries
#include <double_integrator_test/bezier_utilities.hpp>
#include <double_integrator_test/trajectory.hpp>

// CLASSES
class DoubleIntegratorGovernor : public rclcpp::Node {
	public:
		DoubleIntegratorGovernor();
		~DoubleIntegratorGovernor();

	private:
		// FUNCTIONS
		void debugCallback();
		void publishPX4Command(uint16_t command, float param1, float param2);
		void setModeOffboard();
		void arm();
		void disarm();
		void stateMachine();
		void actionCallback(const std_msgs::msg::Empty::SharedPtr msg);
		void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
		void px4StatusCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
		void dynamicsCallback();

		// VARIABLES
		Eigen::Vector3d drone_position;
		Eigen::Vector3d drone_velocity;
		Eigen::Quaterniond drone_orientation;
		Eigen::Vector3d reference_position;
		Eigen::Quaterniond q_reference_yaw;
		double reference_yaw;

		// Trajectory parameters
		double takeoff_altitude;
		
		int debug_timer_frequency_ms;
		int dynamics_timer_frequency_ms;
		int state_machine_timer_frequency_ms;

		// State Variables
		enum governor_state {INIT, ARMED, TAKEOFF, HOVER, ROTATE_RIGHT, MOVE_RIGHT, LAND};
		governor_state agent_state;
		bool message_received;
		bool transition_setpoint_reached;
		bool is_drone_armed;
		bool is_drone_offboard;
		bool follow_trajectory;

		// ROS2 VARIABLES
		
		// Timers
		rclcpp::TimerBase::SharedPtr debug_timer;
		rclcpp::TimerBase::SharedPtr dynamics_timer;
		rclcpp::TimerBase::SharedPtr state_machine_timer;

		// Subscriptions
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;
		rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr action_subscriber;
		rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_state_subscriber;

		// Publishers
		rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher;
		rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_publisher;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_visualizer_publisher;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr setpoint_visualizer_publisher;
		rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher;
};
