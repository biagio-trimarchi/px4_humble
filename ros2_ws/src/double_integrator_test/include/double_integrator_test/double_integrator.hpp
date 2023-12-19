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
		void actionCallback(const std_msgs::msg::Empty msg);
		void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
		void px4StatusCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg);
		void dynamicsCallback();

		// VARIABLES
		Eigen::Vector3d drone_position;
		Eigen::Vector3d drone_velocity;

		double takeoff_altitude;
		double PD_position_gain;
		double PD_velocity_gain;
		Eigen::Vector3d setpoint_position;
		Eigen::Vector3d setpoint_velocity;
		Eigen::Vector3d setpoint_acceleration;
		Trajectory trajectory;
		double trajectory_time;
		double total_time;
		
		int dynamics_timer_frequency_ms;
		int state_machine_timer_frequency_ms;

		// State Variables
		enum governor_state {INIT, ARMED, TAKEOFF, HOVER, FOLLOW_TRAJECTORY, LAND};
		governor_state agent_state;
		bool message_received;
		bool transition_takeoff_reached;
		bool is_drone_armed;
		bool is_drone_offboard;
		bool follow_trajectory;

		// ROS2 VARIABLES
		// Timers
		rclcpp::TimerBase::SharedPtr dynamics_timer;
		rclcpp::TimerBase::SharedPtr state_machine_timer;

		// Subscriptions
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber;
		rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr action_subscriber;
		rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_state_subscriber;

		// Publishers
		rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher;
		rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_publisher;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_visualizer_publisher;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr setpoint_visualizer_publisher;
		rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher;

		// TF2
		std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster; 
};
