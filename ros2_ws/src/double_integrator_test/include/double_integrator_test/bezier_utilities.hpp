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
		BezierCurve(unsigned int _order, int _dimension, double _duration);
		~BezierCurve();

		// Functions
		Eigen::VectorXd evaluate(double time);	
		Eigen::VectorXd evaluate_derivative(double time, int derivative_order);	
		void set_control_point(int index, Eigen::VectorXd &point);

	private:
		// Functions
		Eigen::MatrixXd compute_derivation_matrix(int derivative_order);

		// Variables
		int order;
		int dimension;
		double duration;
		Eigen::MatrixXd control_points;
};
