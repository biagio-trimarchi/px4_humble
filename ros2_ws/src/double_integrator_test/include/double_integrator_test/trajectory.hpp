#pragma once

// LIBRARIES
// C++ Standard Libraries
#include <vector>
#include <memory>
#include <cmath>

// Third Party Libraries
#include <Eigen/Eigen>

// Custom Libraries
#include <double_integrator_test/bezier_utilities.hpp>

class Parameterization {
	// This class implement a parameterization s(t) from the interval [0, T]
	// to the interval [0, 1]
	public:
		// Constructors/Destructor
		Parameterization();
		~Parameterization();

		// Functions
		virtual double evaluate_function(double time);
		virtual double evaluate_first_derivative(double time);
		virtual double evaluate_second_derivative(double time);
	private:
};

class BezierParameterization : public Parameterization {
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
		TrajectorySegment(double _duration);
		~TrajectorySegment();

		// Functions
		Eigen::Vector3d get_position(double time);
		Eigen::Vector3d get_velocity(double time);
		Eigen::Vector3d get_acceleration(double time);
		void set_parameterization(const std::shared_ptr<Parameterization>& new_param);
		void unset_parameterization();
		double get_duration();

	protected:
		// Variables
		double duration;
		bool is_time_parameterized;
		std::shared_ptr<Parameterization> parameterization;
};

class CircleSegment : public TrajectorySegment {
	public:
		// Constructors/destructor
		CircleSegment();
		CircleSegment(double _duration,
                  double _radius,
                  Eigen::Vector3d &_center,
                  double _angular_velocity);
		CircleSegment(double _duration,
                  double _radius,
                  Eigen::Vector3d &_center,
                  double _angular_velocity,
                  Eigen::Matrix3d &_orientation);
		~CircleSegment();

		// Functions
		Eigen::Vector3d get_position(double time);
		Eigen::Vector3d get_velocity(double time);
		Eigen::Vector3d get_acceleration(double time);

	private:
		double radius;
		Eigen::Vector3d center;
		double angular_velocity;
		Eigen::Matrix3d orientation;
};

class SpiralSegment : public TrajectorySegment {
	public:
		// Constructors/destructor
		SpiralSegment();
		SpiralSegment(double _duration,
                  double _height,
                  double _radius,
                  Eigen::Vector3d &_center,
                  double _angular_velocity,
                  bool _is_direction_down);
		SpiralSegment(double _duration,
                  double _height,
                  double _radius,
                  Eigen::Vector3d &_center,
                  double _angular_velocity,
                  bool _is_direction_down,
                  Eigen::Matrix3d &_orientation);
		~SpiralSegment();

		// Functions
		Eigen::Vector3d get_position(double time);
		Eigen::Vector3d get_velocity(double time);
		Eigen::Vector3d get_acceleration(double time);

	private:
		double radius;
		double height;
		Eigen::Vector3d center;
		double angular_velocity;
		bool is_direction_down;
		Eigen::Matrix3d orientation;
};

class BezierSegment : public TrajectorySegment {
	public:
	// Constructors/destructor
	BezierSegment();
	BezierSegment(double _duration,
                unsigned int _order,
                std::vector<Eigen::Vector3d> &control_points);
	~BezierSegment();
	
	// Functions
	Eigen::Vector3d get_position(double time);
	Eigen::Vector3d get_velocity(double time);
	Eigen::Vector3d get_acceleration(double time);

	private:
	BezierCurve bezier_curve;
};

class Trajectory {
	public:
		Trajectory();
		~Trajectory();
	private:
		// Functions
		Eigen::Vector3d evaluate_position(double actual_time);
		Eigen::Vector3d evaluate_velocity(double actual_time);
		Eigen::Vector3d evaluate_acceleration(double actual_time);
		void add_new_segment();
		void delete_trajectory();

		// Variables
		double start_time;
		std::vector<std::unique_ptr<TrajectorySegment*>> segments;
};
