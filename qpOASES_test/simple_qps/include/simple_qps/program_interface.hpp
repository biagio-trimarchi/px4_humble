#pragma once

// C++ Standard Libraries
#include <iostream>
#include <chrono>

// Third Party Libraries
#include <Eigen/Dense>
#include <qpOASES.hpp>

// CLASS
class TestQP {
	public :
		TestQP();
		~TestQP();

		void initialize();
		void run();
	
	private:
		Eigen::Matrix2d H;
		Eigen::Vector2d g;
		Eigen::Vector2d A;
		double lbA[1];
		double ubA[1];
		Eigen::Vector2d lb;
		Eigen::Vector2d ub;

		int nWSR; 
		Eigen::Vector2d x_star;
};

