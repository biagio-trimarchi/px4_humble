// LIBRARIES
#include <double_integrator_test/bezier_utilities.hpp>

// FUNCTIONS IMPLEMENTATION
BezierCurve::BezierCurve() {
	order = 0;
	dimension = 0;
}

BezierCurve::BezierCurve(unsigned int _order, int _dimension, double _duration) {
	order = _order;
	dimension = _dimension;
	duration = _duration;

	control_points = Eigen::MatrixXd::Zero(_dimension, _order+1);
}

BezierCurve::~BezierCurve() {};

void BezierCurve::set_control_point(int index, Eigen::VectorXd &point){
	control_points.col(index) = point;
}

Eigen::VectorXd BezierCurve::evaluate(double time) {
	if (time < 0 || time > duration){
		Eigen::VectorXd result = Eigen::VectorXd::Zero(dimension);

		std::cerr << "[BEZIER CURVE] : Evaluation error, time out of bounds!" << std::endl;
		for (int j = 0; j < dimension; j++) {
			result(j) = std::numeric_limits<double>::quiet_NaN();
		}

		return result;
	}
	
	Eigen::MatrixXd casteljau_points = control_points;

	for (int j=1; j <=order; j++) {
		for (int i=0; i <= order-j; i++){
			casteljau_points.col(i) =  (1 - time / duration) * casteljau_points.col(i)
				                        + (time / duration) * casteljau_points.col(i+1);
		}
	}
	
	return casteljau_points.col(0);
}

Eigen::VectorXd BezierCurve::evaluate_derivative(double time, int derivative_order) {
	if (time < 0 || time > duration){
		Eigen::VectorXd result = Eigen::VectorXd::Zero(dimension);

		std::cerr << "[BEZIER CURVE] : Evaluation error, time out of bounds!" << std::endl;
		for (int j = 0; j < dimension; j++) {
			result(j) = std::numeric_limits<double>::quiet_NaN();
		}

		return result;
	}

	Eigen::MatrixXd derivation_matrix = compute_derivation_matrix(derivative_order);
	Eigen::MatrixXd derivative_control_points = control_points;
	derivative_control_points = derivative_control_points * derivation_matrix;
	int number_derivative_control_points = static_cast<int>(derivative_control_points.cols());

	// Evaluate using De Casteljau
	for (int j=1; j < number_derivative_control_points; j++) {
		for (int i=0; i < number_derivative_control_points-j; i++){
			derivative_control_points.col(i) =  (1 - time / duration) * derivative_control_points.col(i)
				                        + (time / duration) * derivative_control_points.col(i+1);
		}
	}
		
	return derivative_control_points.col(0);
}

Eigen::MatrixXd BezierCurve::compute_derivation_matrix(int derivative_order) {
	Eigen::MatrixXd result = Eigen::MatrixXd::Identity(order+1, order+1);

	if (derivative_order < 0) {
		std::cerr << "[BEZIER CURVE] : Evaluation error, requested derivative order is less than 0!" << std::endl;
		for (int row=0; row < order+1; row++)
			for (int col=0; col < order+1; col++)
				result(row, col) = std::numeric_limits<double>::quiet_NaN();

		return result;
	}

	if (derivative_order > order) 
		return Eigen::VectorXd::Zero(order+1); 

	for (int i=1; i<=derivative_order; i++) {
		Eigen::MatrixXd single_derivation_matrix = Eigen::MatrixXd::Zero(order+2-i , order+1-i);
		for (int j=0; j < order+1-i; j++) {
			single_derivation_matrix(j, j) = -1.0 / duration * double(order - i + 1);
			single_derivation_matrix(j+1, j) = 1.0 / duration * double(order - i + 1);
		}
		result =  result * single_derivation_matrix;
	}

	return result;
}
