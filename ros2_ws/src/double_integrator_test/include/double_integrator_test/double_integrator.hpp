#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

// C++ Standard Libraries
#include <chrono>

// Third Party Libraries
#include <Eigen/Eigen>

// CLASSES
class DoubleIntegratorGovernor : public rclcpp::Node {
	public:
		DoubleIntegratorGovernor();
		~DoubleIntegratorGovernor();

	private:
		// FUNCTIONS
		void stateMachine();
		void actionCallback(const std_msgs::msg::Empty msg);
		void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
		void dynamicsCallback();

		// VARIABLES
		Eigen::Vector3d reference_position;
		Eigen::Vector3d reference_velocity;
		Eigen::Vector3d reference_acceleration;

		Eigen::Vector3d drone_position;

		double takeoff_altitude;
		double PD_position_gain;
		double PD_velocity_gain;
		Eigen::Vector3d setpoint_position;
		Eigen::Vector3d setpoint_velocity;
		Eigen::Vector3d setpoint_acceleration;
		
		int dynamics_timer_frequency_ms;
		int state_machine_timer_frequency_ms;

		// State Variables
		enum governor_state {INIT, TAKEOFF, HOVER, CIRCLE, SPIRAL_UP, SPIRAL_DOWN, LAND};
		governor_state agent_state;
		bool transition_takeoff;
		bool transition_takeoff_reached;

		// ROS2 VARIABLES
		// Timers
		rclcpp::TimerBase::SharedPtr dynamics_timer;
		rclcpp::TimerBase::SharedPtr state_machine_timer;

		// Subscriptions
		rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscriber;
		rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr action_subscriber;

		// Publishers
		rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_publisher;
		rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_publisher;
};
