#pragma once

// LIBRARIES
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

// C++ Standard Libraries
#include <chrono>

// Third Party Libraries
#include <Eigen/Eigen>
#include <qpOASES.hpp>

// Custom Libraries
#include <log_gpis/logGPIS.hpp>

// CLASSES
class Test : public rclcpp::Node {
	public:
		Test();
		~Test();

	private:
		// FUNCTIONS
		void addObstacles();
		void controller();
		void barrierFunction();
		void dynamics();
		void simulate();
		void debugCallback();

	
		// VARIABLES
		Eigen::Vector3d reference_position;
		Eigen::Vector3d reference_velocity;
		Eigen::Vector3d reference_acceleration;

		Eigen::Vector3d setpoint_position;
		Eigen::Vector3d setpoint_velocity;

		double delta_t;
		double controller_P_gain;
		double controller_D_gain;

		qpOASES::QProblem qpOASES_solver;
		qpOASES::Options qpOASES_options;
		Eigen::Matrix3d qpOASES_H;
		Eigen::Vector3d qpOASES_g;
		Eigen::MatrixXd qpOASES_A;
		Eigen::VectorXd qpOASES_lbA;
		Eigen::VectorXd qpOASES_ubA;
		Eigen::Vector3d qpOASES_lb;
		Eigen::Vector3d qpOASES_ub;
		int qpOASES_constraints_number;
		int qpOASES_nWSR;

		double bf_classK_gain_1;
		double bf_classK_gain_2;
		double bf_gain_lie_0_kh;
		double bf_gain_lie_1_kh;
		double bf_safe_margin;

		visualization_msgs::msg::Marker trajectory_msg;
		visualization_msgs::msg::Marker obstacle_msg;

		// Timers
		rclcpp::TimerBase::SharedPtr timer_visualization;

		// Publishers
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_trajectory;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_obstacles;
};
