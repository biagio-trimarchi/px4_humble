#pragma once

// LIBRARIES
// C++ Standard Libraries
#include <vector>
#include <memory>

// Third Party Libraries
#include <Eigen/Eigen>

// Custom Libraries
#include <double_integrator_test/bezier_utilities.hpp>

class Parameterization {
	public:
		// Constructors/Destructor
		Parameterization();
		~Parameterization();

		// Functions
		double evaluate_function(double time);
		double evaluate_first_derivative(double time);
		double evaluate_second_derivative(double time);
	private:
};

class BezierParameterization : Parameterization {
	public:
		// Constructors/Destructor
		BezierParameterization();
		BezierParameterization(unsigned int order, const std::vector<float> control_points, double duration);
		~BezierParameterization();

		// Functions
		double evaluate_function(double time);
		double evaluate_first_derivative(double time);
		double evaluate_second_derivative(double time);
	private:
		BezierCurve scalar_Bezier;
};

class TrajectorySegment {
	public:
		// Constructors/destructor
		TrajectorySegment();
		TrajectorySegment(double _duration, unsigned int _dimension);
		Eigen::Vector3d get_velocity(double time);
		Eigen::Vector3d get_acceleration(double time);
		void set_parameterization(const std::shared_ptr<Parameterization> new_param);
		void unset_parameterization();

	protected:
		// Variables
		double duration;
		unsigned int dimension;
		bool is_time_parameterized;
		std::shared_ptr<Parameterization> parameterization;
};

class CircleSegment : TrajectorySegment {
	public:
		// Constructors/destructor
		CircleSegment();
		CircleSegment(double _duration,
                  unsigned int _dimension, 
                  Eigen::Vector3d &_radius,
                  Eigen::Vector3d &_center,
									Eigen::Matrix3d &_orientation);
		~CircleSegment();

		// Functions
		Eigen::Vector3d get_position(double time);
		Eigen::Vector3d get_velocity(double time);
		Eigen::Vector3d get_acceleration(double time);

	private:
		Eigen::Vector3d radius;
		Eigen::Vector3d center;
		Eigen::Matrix3d orientation;
}

class Trajectory {
	public:
		Trajectory();
		~Trajectory();
	private:
		// Functions
		Eigen::Vector3d evaluate_trajectory_position(double actual_time);
		Eigen::Vector3d evaluate_trajectory_velocity(double actual_time);
		Eigen::Vector3d evaluate_trajectory_acceleration(double actual_time);
		void add_new_segment();
		void delete_trajectory();

		// Variables
		double start_time;
		std::vector<std::unique_ptr<TrajectorySegment*>> segments;
};
