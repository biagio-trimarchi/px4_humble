// LIBRARIES
#include <log_gpis/logGPIS.hpp>

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

Eigen::MatrixXd LogGPIS::getK() {
	return gaussian_process.getK();
}

Eigen::MatrixXd LogGPIS::getAlpha() {
	return gaussian_process.getAlpha();
}

Eigen::MatrixXd LogGPIS::getDataX() {
	return gaussian_process.getDataX();
}

Eigen::MatrixXd LogGPIS::getDataY() {
	return gaussian_process.getDataY();
}

void LogGPIS::loadData(const Eigen::MatrixXd& inputK,
                       const Eigen::MatrixXd& input_alpha,
                       const Eigen::MatrixXd& input_data_x,
                       const Eigen::VectorXd& input_data_y) {
	gaussian_process.loadData(inputK, input_alpha, input_data_x, input_data_y);
}

