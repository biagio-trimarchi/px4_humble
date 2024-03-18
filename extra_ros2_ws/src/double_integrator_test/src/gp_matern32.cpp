// LIBRARIES
#include <double_integrator_test/gp_matern32.hpp>

// CLASSES
// GAUSSIAN PROCESS MATERN 32
// Public Functions
GaussianProcessMatern32::GaussianProcessMatern32() {}

GaussianProcessMatern32::GaussianProcessMatern32(double length_scale,
                                                 double error_variance,
																								 double resolution) {
	this->length_scale = length_scale;
	this->error_variance = error_variance;
	this->resolution = resolution;
	number_of_samples = 0;
}

GaussianProcessMatern32::~GaussianProcessMatern32() {}

// Public Functions
void GaussianProcessMatern32::add_sample(Eigen::Vector3d x, double y) {
	x /= length_scale;

	if (number_of_samples == 0) {
		data_x.resize(3, 1);
		data_y = Eigen::VectorXd(1);

		data_x.col(0) = x;
		data_y(0) = y;
	} else {

		if (check_collected(x))
			return;

		data_x.conservativeResize(Eigen::NoChange, number_of_samples + 1);
		data_y.conservativeResize(number_of_samples + 1);

		data_x.col(number_of_samples) = x;
		data_y(number_of_samples) = y;
	}
	number_of_samples++;
}

void GaussianProcessMatern32::train() {
	Eigen::MatrixXd L_cholesky;

	K = Eigen::MatrixXd::Zero(number_of_samples, number_of_samples);
	for (unsigned int row = 0; row < number_of_samples; row++)
		for (unsigned int col = 0; col < number_of_samples; col++) 
			K(row, col) = kernel(data_x.col(row), data_x.col(col));
	
	K += error_variance * Eigen::MatrixXd::Identity(number_of_samples, number_of_samples);

	L_cholesky = K.llt().matrixL();
alpha = L_cholesky.transpose().colPivHouseholderQr().solve(L_cholesky.colPivHouseholderQr().solve(data_y));
 
}

double GaussianProcessMatern32::posterior_mean(Eigen::Vector3d x) {
	Eigen::RowVectorXd k(number_of_samples);

	x /= length_scale;
	for (unsigned int i = 0; i < number_of_samples; i++)
		k(i) = kernel(x, data_x.col(i));
	
	return k * alpha;
}

Eigen::RowVector3d GaussianProcessMatern32::gradient_posterior_mean(Eigen::Vector3d x) {
	Eigen::MatrixX3d dk;
	dk.resize(number_of_samples, 3);

	x /= length_scale;
	for (unsigned int i = 0; i < number_of_samples; i++)
		dk.row(i) = gradient_kernel(x, data_x.col(i));

	return alpha.transpose() * dk;
}

Eigen::Matrix3d GaussianProcessMatern32::hessian_posterior_mean(Eigen::Vector3d x) {
	Eigen::Matrix3d auxiliary_sum = Eigen::Matrix3d::Zero();
	
	x /= length_scale;
	for (unsigned int i = 0; i < number_of_samples; i++)
		auxiliary_sum += alpha(i) * hessian_kernel(x, data_x.col(i));
	
	return auxiliary_sum;
}

double GaussianProcessMatern32::posterior_variance(Eigen::Vector3d x) {
	Eigen::VectorXd k(number_of_samples);

	x /= length_scale;
	for (unsigned int i = 0; i < number_of_samples; i++)
		k(i) = kernel(x, data_x.col(i));

	Eigen::VectorXd aux = K.fullPivLu().solve(k);
	return kernel(x, x) - k.transpose() * aux;
}

Eigen::RowVector3d GaussianProcessMatern32::gradient_posterior_variance(Eigen::Vector3d x) {
	Eigen::VectorXd k(number_of_samples);
	Eigen::MatrixXd dk(number_of_samples, number_of_samples);

	x /= length_scale;
	for (unsigned int i = 0; i < number_of_samples; i++) {
		k(i) = kernel(x, data_x.col(i));
		dk.row(i) = gradient_kernel(x, data_x.col(i));
	}

	Eigen::VectorXd aux = K.fullPivLu().solve(k);
	return -2.0 * k.transpose() * aux.transpose() * dk;
}

// Private Functions
bool GaussianProcessMatern32::check_collected(Eigen::Vector3d x) {
	if (number_of_samples == 0)
		return false;

	for (int col = 0; col < data_x.cols(); col++) {
		Eigen::Vector3d collected_point = data_x.col(col) * length_scale;
		if ((x - collected_point).norm() < resolution)
			return true;
	}
	return false;
}

double GaussianProcessMatern32::kernel(Eigen::Vector3d x_1, 
                                       Eigen::Vector3d x_2) {
	double r = (x_1 - x_2).norm();

	return (1 + std::sqrt(3.0) * r) * std::exp(-std::sqrt(3.0) * r);
}

Eigen::RowVector3d GaussianProcessMatern32::gradient_kernel(Eigen::Vector3d x_1,
                                                            Eigen::Vector3d x_2) {
	double r = (x_1 - x_2).norm();

	return -3.0 * (x_1 - x_2).transpose() * std::exp(-std::sqrt(3.0) * r);
}

Eigen::Matrix3d GaussianProcessMatern32::hessian_kernel(Eigen::Vector3d x_1,
		                                                    Eigen::Vector3d x_2) {
	double r = (x_1 - x_2).norm();

	if (r < 1.0e-3) {
		return -3.0 * Eigen::Matrix3d::Identity() * std::exp(-std::sqrt(3.0) * r);
	}

	return (-3.0 * Eigen::Matrix3d::Identity() + 
	         3.0 * std::sqrt(3.0) * ((x_1 - x_2) *  (x_1 - x_2).transpose()) / r ) * 
	         std::exp(-std::sqrt(3.0) * r);
}
