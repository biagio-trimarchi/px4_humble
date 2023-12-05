#pragma once

// LIBRARIES
// C++ Standard Libraries

// Third Party Libraries
#include <Eigen/Eigen>

class Trajectory {
	public:
		Trajectory();
		~Trajectory();
	private:
		// Functions
		Eigen::Vector3d evaluate_trajectory(double actual_time);

		// Variables
		double start_time;
};

class BezierMultiSegment : Trajectory {
}

Eigen::Vector3d circular_trajectory(double current_time,
		                                double initial_time,
		                                double time_segment_forward, 
		                                double time_circle,
																		double time_segment_backward);
