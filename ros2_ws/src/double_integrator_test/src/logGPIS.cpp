// LIBRARIES
#include <double_integrator_test/logGPIS.hpp>

// CLASSES
// LogGPIS
LogGPIS::LogGPIS() {};

LogGPIS::LogGPIS(double lambda_whittle, double resolution, 
                 double error_variance) {
	this->lambda_whittle = lambda_whittle;
	this->resolution = resolution;

	gaussian_process = GaussianProcessMatern32(std::sqrt(3.0) / lambda_whittle, 
	                                           error_variance, resolution);
}

LogGPIS::~LogGPIS() {};

void LogGPIS::add_sample(Eigen::Vector3d x) {
	gaussian_process.add_sample(x, 1.0);
}

void LogGPIS::train() {
	gaussian_process.train();
}

double LogGPIS::evaluate(Eigen::Vector3d x) {
	return -std::log(gaussian_process.posterior_mean(x)) / lambda_whittle;	
}

Eigen::RowVector3d LogGPIS::gradient(Eigen::Vector3d x) {
	double d = evaluate(x);
	Eigen::RowVector3d grad = gaussian_process.gradient_posterior_mean(x);

	return -grad / (lambda_whittle * d);
}

Eigen::Matrix3d LogGPIS::hessian(Eigen::Vector3d x) {
	double d = evaluate(x);
	Eigen::RowVector3d grad = gaussian_process.gradient_posterior_mean(x);
	Eigen::Matrix3d hess = gaussian_process.hessian_posterior_mean(x);
	
	Eigen::Matrix3d a = grad.transpose() * grad / (lambda_whittle * d * d);
	Eigen::Matrix3d b = hess / (lambda_whittle * d);
	return a - b;
}
