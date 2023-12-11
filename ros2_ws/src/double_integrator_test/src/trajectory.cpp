// LIBRARIES
#include <double_integrator_test/trajectory.hpp>

// FUNCTIONS

// Parameterization
// Identity parameterization
Parameterization::Parameterization() {}

Parameterization::~Parameterization() {}

double Parameterization::evaluate_function(double time) {
	return time;
}

double Parameterization::evaluate_first_derivative(double time) {
	return 0;
}

double Parameterization::evaluate_second_derivative(double time) {
	return 0;
}

// Bezier curve parameterization
BezierParameterization::BezierParameterization() {}

BezierParameterization::BezierParameterization(unsigned int order, const std::vector<float> control_points, double duration) : Parameterization() {
	if (control_points.size() != order+1) {
		std::cerr << "[ERROR] (BezierParameterization constructor) : 'order' do not match the size of 'control_points'" << std::endl;;
	}

	scalar_Bezier = BezierCurve(order, 1, duration);
	for (unsigned int i = 0; i < order+1; i++){
		Eigen::VectorXd point {{control_points[i]}};
		scalar_Bezier.set_control_point(i, point);
	}
}

BezierParameterization::~BezierParameterization() {}

double BezierParameterization::evaluate_function(double time) {
	return (scalar_Bezier.evaluate(time))(0);
}

double BezierParameterization::evaluate_first_derivative(double time) {
	return (scalar_Bezier.evaluate_derivative(time, 1))(0);
}

double BezierParameterization::evaluate_second_derivative(double time) {
	return (scalar_Bezier.evaluate_derivative(time, 2))(0);
}

// Trajectory segment
// Base template
TrajectorySegment::TrajectorySegment() {}

TrajectorySegment::TrajectorySegment(double _duration, unsigned int _dimension) {
	duration = _duration;
	dimension = _dimension;
	is_time_parameterized = false;
}

TrajectorySegment::~TrajectorySegment() {}

TrajectorySegment::set_parameterization(const std::shared_ptr<Parameterization> new_param) {
	parameterization = new_param;
	is_time_parameterized = true;
}

TrajectorySegment::unset_parameterization() {
	parameterization.reset();
	is_time_parameterized = false;
}

// Circle Segment
CircleSegment::CircleSegment() {}

CircleSegment::CircleSegment(double _duration, 
                             unsigned int _dimension,
                             Eigen::VectorXd &_radius,
														 Eigen::VectorXd &_center,
                             double angular_velocity,
                             Eigen::Matrix3d &_orientation) : TrajectorySegment(_duration, _dimension) {
	radius = _radius;
	center = _center;
	orientation = _orientation;
}

Eigen::Vector3d CircleSegment::get_position(double time) {
	Eigen::Vector3d result = Eigen::VectorXd::Zero(dimension);

		
}
