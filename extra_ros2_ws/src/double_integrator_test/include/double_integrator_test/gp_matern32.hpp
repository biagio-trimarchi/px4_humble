#pragma once
// LIBRARIES
// C++ Standard Libraries
#include <cmath>

// Third Party Libraries
#include <Eigen/Dense>

// CLASSES
class GaussianProcessMatern32 {
	public:
		// Constructors/Destructor
		GaussianProcessMatern32();
		GaussianProcessMatern32(double length_scale, 
		                        double error_variance,
														double resolution);
		~GaussianProcessMatern32();
		// Functions
		void add_sample(Eigen::Vector3d x, double y);
		void train();
		double posterior_mean(Eigen::Vector3d x);
		Eigen::RowVector3d gradient_posterior_mean(Eigen::Vector3d x);
		Eigen::Matrix3d hessian_posterior_mean(Eigen::Vector3d x);
		double posterior_variance(Eigen::Vector3d x);
		Eigen::RowVector3d gradient_posterior_variance(Eigen::Vector3d x);
	
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
