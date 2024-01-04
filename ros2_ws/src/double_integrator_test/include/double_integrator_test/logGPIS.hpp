# pragma once
// LIBRARIES
// C++ Standard Libraries
#include <cmath>

// Third Pary Libraries
#include <Eigen/Dense>

// Custom Libraries
#include <double_integrator_test/gp_matern32.hpp>

// CLASSES
class LogGPIS {
	public:
		// Constructors/Destructor
		LogGPIS();
		LogGPIS(double lambda_whittle, double resolution, double error_variance);
		~LogGPIS();

		// Functions
		void add_sample(Eigen::Vector3d x);
		void train();
		double evaluate(Eigen::Vector3d x);
		Eigen::RowVector3d gradient(Eigen::Vector3d x);
		Eigen::Matrix3d hessian(Eigen::Vector3d x);

	private:
		// Functions

		// Variables
		double lambda_whittle;
		double resolution;
		GaussianProcessMatern32 gaussian_process;
};
