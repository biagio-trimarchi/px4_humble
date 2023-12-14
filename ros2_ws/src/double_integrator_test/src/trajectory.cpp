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

TrajectorySegment::TrajectorySegment(double _duration) {
	duration = _duration;
	is_time_parameterized = false;
}

TrajectorySegment::~TrajectorySegment() {}

void TrajectorySegment::set_parameterization(const std::shared_ptr<Parameterization>& new_param) {
	parameterization = new_param;
	is_time_parameterized = true;
}

void TrajectorySegment::unset_parameterization() {
	parameterization.reset();
	is_time_parameterized = false;
}

// Circle Segment
CircleSegment::CircleSegment() {}

CircleSegment::CircleSegment(double _duration, 
                             double _radius,
														 Eigen::Vector3d &_center,
                             double _angular_velocity) : TrajectorySegment(_duration) {
	radius = _radius;
	center = _center;
	angular_velocity = _angular_velocity;
	orientation = Eigen::Matrix3d::Identity();
}

CircleSegment::CircleSegment(double _duration, 
                             double _radius,
														 Eigen::Vector3d &_center,
                             double _angular_velocity,
                             Eigen::Matrix3d &_orientation) : TrajectorySegment(_duration) {
	radius = _radius;
	center = _center;
	angular_velocity = _angular_velocity;
	orientation = _orientation;
}

CircleSegment::~CircleSegment() {}

Eigen::Vector3d CircleSegment::get_position(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time;

	if (time < 0 || time > duration) {
		std::cerr << "[Trajectory Segment] (get_position) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
	}

	result.x() = center.x() + radius * cos(angular_velocity * s);	
	result.y() = center.y() + radius * sin(angular_velocity * s);	
	return orientation * result;
}

Eigen::Vector3d CircleSegment::get_velocity(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time;
	double ds = 1.0;

	if (time < 0 || time > duration) {
		std::cerr << "[Trajectory Segment] (get_velocity) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
		ds = parameterization->evaluate_first_derivative(time);
	}
	
	result.x() = - ds * angular_velocity * radius * sin(angular_velocity * s);
	result.y() =   ds * angular_velocity * radius * cos(angular_velocity * s);

	return orientation * result;
}

Eigen::Vector3d CircleSegment::get_acceleration(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time;
	double ds = 1.0;
	double dds = 1.0;

	if (time < 0 || time > duration) {
		std::cerr << "[Trajectory Segment] (get_acceleration) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
		ds = parameterization->evaluate_first_derivative(time);
		dds = parameterization->evaluate_second_derivative(time);
	}
	
	result.x() = - pow(ds * angular_velocity, 2.0) * radius * cos(angular_velocity * s) - 
	                        dds * angular_velocity * radius * sin(angular_velocity * s);
	result.y() = - pow(ds * angular_velocity, 2.0) * radius * sin(angular_velocity * s) + 
		                      dds * angular_velocity * radius * cos(angular_velocity * s);

	return orientation * result;
}

// Bezier Segment
