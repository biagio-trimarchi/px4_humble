#pragma once

// LIBRARIES
// C++ Standard Libraries
#include <vector>
#include <iostream>
#include <limits>

// Third Party Libraries
#include <Eigen/Eigen>

class BezierCurve {
	public:
		// Constructors - Destructor
		BezierCurve();
		BezierCurve(int _order, int _dimension, double _duration);
		~BezierCurve();

		// Functions
		Eigen::VectorXd evaluate(double time);	
		void set_control_point(int index, Eigen::VectorXd &point);

	private:
		// Variables
		int order;
		int dimension;
		double duration;
		Eigen::MatrixXd control_points;
};
