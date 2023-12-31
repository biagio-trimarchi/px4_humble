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

BezierParameterization::BezierParameterization(unsigned int order, const std::vector<double> &control_points, double duration) : Parameterization() {
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

double TrajectorySegment::get_duration() {
	return duration;
}

Eigen::Vector3d TrajectorySegment::get_position(double time) {
	return Eigen::Vector3d::Zero();;
}

Eigen::Vector3d TrajectorySegment::get_velocity(double time) {
	return Eigen::Vector3d::Zero();;
}

Eigen::Vector3d TrajectorySegment::get_acceleration(double time) {
	return Eigen::Vector3d::Zero();;
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
	double s = time / duration;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Circle Segment] (get_position) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
	}

	result.x() = center.x() + radius * cos(angular_velocity * duration * s);	
	result.y() = center.y() + radius * sin(angular_velocity * duration * s);	
	result.z() = center.z();
	return orientation * result;
}

Eigen::Vector3d CircleSegment::get_velocity(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time / duration;
	double ds = 1.0 / duration;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Circle Segment] (get_velocity) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
		ds = parameterization->evaluate_first_derivative(time);
	}
	
	result.x() = - ds * angular_velocity * duration * radius * sin(angular_velocity * duration * s);
	result.y() =   ds * angular_velocity * duration * radius * cos(angular_velocity * duration * s);

	return orientation * result;
}

Eigen::Vector3d CircleSegment::get_acceleration(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time / duration;
	double ds = 1.0 / duration;
	double dds = 0.0;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Circle Segment] (get_acceleration) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
		ds = parameterization->evaluate_first_derivative(time);
		dds = parameterization->evaluate_second_derivative(time);
	}
	
	result.x() = - pow(ds * angular_velocity * duration, 2.0) * radius * cos(angular_velocity * duration * s) - 
	                        dds * angular_velocity * duration * radius * sin(angular_velocity * duration * s);
	result.y() = - pow(ds * angular_velocity * duration, 2.0) * radius * sin(angular_velocity * duration * s) + 
		                      dds * angular_velocity * duration * radius * cos(angular_velocity * duration * s);

	return orientation * result;
}

// Spiral Segment
SpiralSegment::SpiralSegment() {};

SpiralSegment::SpiralSegment(double _duration,
                             double _height,
                             double _radius,
                             Eigen::Vector3d &_center,
                             double _angular_velocity,
                             bool _is_direction_down) : TrajectorySegment(_duration) {
	height = _height;
	radius = _radius;
	center = _center;
	angular_velocity = _angular_velocity;
	is_direction_down = _is_direction_down;
	orientation = Eigen::Matrix3d::Identity();
}

SpiralSegment::SpiralSegment(double _duration,
                             double _height,
                             double _radius,
                             Eigen::Vector3d &_center,
                             double _angular_velocity,
                             bool _is_direction_down,
                             Eigen::Matrix3d &_orientation) : TrajectorySegment(_duration) {
	height = _height;
	radius = _radius;
	center = _center;
	angular_velocity = _angular_velocity;
	is_direction_down = _is_direction_down;
	orientation = _orientation;
}

SpiralSegment::~SpiralSegment() {};

Eigen::Vector3d SpiralSegment::get_position(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time / duration;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Spiral Segment] (get_position) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
	}

	result.x() = center.x() + radius * cos(angular_velocity * duration * s);	
	result.y() = center.y() + radius * sin(angular_velocity * duration * s);	
	result.z() = center.z() + height * s * (is_direction_down?-1.0:1.0);
	return orientation * result;
}

Eigen::Vector3d SpiralSegment::get_velocity(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time / duration;
	double ds = 1.0 / duration;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Spiral Segment] (get_velocity) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
		ds = parameterization->evaluate_first_derivative(time);
	}
	
	result.x() = - ds * angular_velocity * duration * radius * sin(angular_velocity * duration * s);
	result.y() =   ds * angular_velocity * duration * radius * cos(angular_velocity * duration * s);
	result.z() =   height * ds * (is_direction_down?-1.0:1.0);

	return orientation * result;
}

Eigen::Vector3d SpiralSegment::get_acceleration(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time / duration;
	double ds = 1.0 / duration;
	double dds = 0.0;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Spiral Segment] (get_acceleration) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
		ds = parameterization->evaluate_first_derivative(time);
		dds = parameterization->evaluate_second_derivative(time);
	}
	
	result.x() = - pow(ds * angular_velocity * duration, 2.0) * radius * cos(angular_velocity * duration * s) - 
	                        dds * angular_velocity * duration * radius * sin(angular_velocity * duration * s);
	result.y() = - pow(ds * angular_velocity * duration, 2.0) * radius * sin(angular_velocity * duration * s) + 
		                      dds * angular_velocity * duration * radius * cos(angular_velocity * duration * s);
	result.z() =   height * dds * (is_direction_down?-1.0:1.0);

	return orientation * result;
}

// Bezier Segment
BezierSegment::BezierSegment() {}

BezierSegment::BezierSegment(double _duration,
                             unsigned int _order,
                             std::vector<Eigen::Vector3d> &control_points) : TrajectorySegment(_duration) {
	bezier_curve = BezierCurve(_order, 3, _duration);
	
	if (control_points.size() != _order+1) {
		std::cerr << "[ERROR] (BezierSegment constructor) : 'order' do not match the size of 'control_points'" << std::endl;;
	}

	for (unsigned int i = 0; i < control_points.size(); i++) {
		Eigen::VectorXd point = Eigen::VectorXd::Zero(3);
		point(0) = control_points[i].x();
		point(1) = (control_points[i]).y();
		point(2) = (control_points[i]).z();
		bezier_curve.set_control_point(i, point);
	}
}

BezierSegment::~BezierSegment() {}

Eigen::Vector3d BezierSegment::get_position(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time / duration;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Bezier Segment] (get_position) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
	}

	result = bezier_curve.evaluate(duration * s); 
	return result;
}

Eigen::Vector3d BezierSegment::get_velocity(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time / duration;
	double ds = 1.0 / duration;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Bezier Segment] (get_velocity) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
		ds = parameterization->evaluate_first_derivative(time);
	}

	result = ds * duration *  bezier_curve.evaluate_derivative(duration * s, 1);
	return result;
}

Eigen::Vector3d BezierSegment::get_acceleration(double time) {
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	double s = time / duration;
	double ds = 1.0 / duration;
	double dds = 0.0;

	if (time < 0.0 || time > duration) {
		std::cerr << "[Bezier Segment] (get_acceleration) : 'time' out of bound" << std::endl;
		return result;
	}

	if (is_time_parameterized) {
		s = parameterization->evaluate_function(time);
		s = parameterization->evaluate_first_derivative(time);
		ds = parameterization->evaluate_second_derivative(time);
	}

	result = dds * duration * bezier_curve.evaluate_derivative(duration * s, 1) + pow(ds * duration, 2.0) *  bezier_curve.evaluate_derivative(duration * s, 2);
	return result;
}

// Trajectory
Trajectory::Trajectory() {}

Trajectory::~Trajectory() {}

Eigen::Vector3d Trajectory::evaluate_position(double time) {
	Eigen::Vector3d result;

	unsigned int segment_index = 0;	
	double cumulated_segments_time = segments[segment_index]->get_duration();
	for (; segment_index < segments.size() && cumulated_segments_time < time;) {
		segment_index++;
		cumulated_segments_time += segments[segment_index]->get_duration();
	}
	if (cumulated_segments_time < time) {
		std::cerr << "[Trajectory] (evaluate_position) : 'time' out of bound" << std::endl;
		return result;
	}

	cumulated_segments_time -= segments[segment_index]->get_duration();
	result = segments[segment_index]->get_position(time - cumulated_segments_time);

	return result;	
}

Eigen::Vector3d Trajectory::evaluate_velocity(double time) {
	Eigen::Vector3d result;

	unsigned int segment_index = 0;	
	double cumulated_segments_time = segments[segment_index]->get_duration();
	for (; segment_index < segments.size() && cumulated_segments_time < time;) {
		segment_index++;
		cumulated_segments_time += segments[segment_index]->get_duration();
	}
	if (cumulated_segments_time < time) {
		std::cerr << "[Trajectory] (evaluate_position) : 'time' out of bound" << std::endl;
		return result;
	}

	cumulated_segments_time -= segments[segment_index]->get_duration();
	result = segments[segment_index]->get_velocity(time - cumulated_segments_time);

	return result;	
}

Eigen::Vector3d Trajectory::evaluate_acceleration(double time) {
	Eigen::Vector3d result;

	unsigned int segment_index = 0;	
	double cumulated_segments_time = segments[segment_index]->get_duration();
	for (; segment_index < segments.size() && cumulated_segments_time < time;) {
		segment_index++;
		cumulated_segments_time += segments[segment_index]->get_duration();
	}
	if (cumulated_segments_time < time) {
		std::cerr << "[Trajectory] (evaluate_position) : 'time' out of bound" << std::endl;
		return result;
	}

	cumulated_segments_time -= segments[segment_index]->get_duration();
	result = segments[segment_index]->get_acceleration(time - cumulated_segments_time);

	return result;	
}

void Trajectory::add_segment(const std::shared_ptr<TrajectorySegment> &segment, unsigned int index) {
	if (index > segments.size()) {
		std::cerr << "[Trajectory] (add_segment) : 'index' out of bound, segment not inserted " << std::endl;
		return;
	}

	auto iterator = segments.begin() + index;
	segments.insert(iterator, segment);
}

void Trajectory::append_segment(const std::shared_ptr<TrajectorySegment> &segment) {
	segments.push_back(segment);
}


void Trajectory::delete_segment(unsigned int index) {
	if (index > segments.size()) {
		std::cerr << "[Trajectory] (remove_segment) : 'index' out of bound, segment not deleted" << std::endl;
		return;
	}

	auto iterator = segments.begin() + index;
	segments.erase(iterator);
}

void Trajectory::delete_trajectory() {
	segments.clear();	
}
