#pragma once
// LIBRARIES
// C++ Standard Libraries
#include <iostream>
#include <cmath>

// Third Party Libraries
#include <Eigen/Dense>

// CLASSES
class GaussianProcessMatern12 {
	public:
		// Constructors/Destructor
		GaussianProcessMatern12();
		GaussianProcessMatern12(double length_scale, 
		                        double error_variance,
                                        double resolution);
		~GaussianProcessMatern12();
		// Functions
		void add_sample(Eigen::Vector3d x, double y);
		void train();
		double posterior_mean(Eigen::Vector3d x);
		Eigen::RowVector3d gradient_posterior_mean(Eigen::Vector3d x);
		Eigen::Matrix3d hessian_posterior_mean(Eigen::Vector3d x);
		double posterior_variance(Eigen::Vector3d x);
		Eigen::RowVector3d gradient_posterior_variance(Eigen::Vector3d x);

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
		bool check_collected(Eigen::Vector3d x);
		double kernel(Eigen::Vector3d x_1, Eigen::Vector3d x_2);
		Eigen::RowVector3d gradient_kernel(Eigen::Vector3d x_1, Eigen::Vector3d x_2);
		Eigen::Matrix3d hessian_kernel(Eigen::Vector3d x_1, Eigen::Vector3d x_2);

		// Variables
		double error_variance;
		double length_scale;
		double resolution;
		unsigned int number_of_samples;

		Eigen::Matrix3Xd data_x;
		Eigen::VectorXd data_y;
		Eigen::MatrixXd K;
		Eigen::VectorXd alpha;
};
