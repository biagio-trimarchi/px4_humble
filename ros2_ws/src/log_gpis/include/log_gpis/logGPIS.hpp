# pragma once
// LIBRARIES
// C++ Standard Libraries
#include <cmath>

// Third Pary Libraries
#include <Eigen/Dense>

// Custom Libraries
#include <log_gpis/gp_matern32.hpp>

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

		// Get and Set
		Eigen::MatrixXd getK();
		Eigen::MatrixXd getAlpha();
		Eigen::MatrixXd getDataX();
		Eigen::MatrixXd getDataY();

		void loadData(const Eigen::MatrixXd& inputK,
                  const Eigen::MatrixXd& input_alpha,
                  const Eigen::MatrixXd& input_data_x,
                  const Eigen::VectorXd& input_data_y);


	private:
		// Functions

		// Variables
		double lambda_whittle;
		double resolution;
		GaussianProcessMatern32 gaussian_process;
};
