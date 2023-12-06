// LIBRARIES
#include <double_integrator_test/bezier_utilities.hpp>

BezierCurve::BezierCurve() {
	order = 0;
	dimension = 0;
}

BezierCurve::BezierCurve(int _order, int _dimension, double _duration) {
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
