#pragma once

// LIBRARIES
// C++ Standard Libraries
#include <vector>

// Third Party Libraries
#include <Eigen/Eigen>

class Parameterization {
	public:
		Parameterization();
		~Parameterization();
	private:
		// Functions
		void evaluate_function();
		void evaluate_first_derivative();
		void evalutate_second_derivative();
};

class TrajectorySegment {
	public:
		TrajectorySegment(double _duration, bool _is_time_parameterized);
		~TrajectorySegment();
	private:
		// Functions

		// Variables
		double duration;
		bool is_time_parameterized;
		std::unique_ptr<Parameterization> parameterization;
};

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

class BezierMultiSegment : Trajectory {
};
