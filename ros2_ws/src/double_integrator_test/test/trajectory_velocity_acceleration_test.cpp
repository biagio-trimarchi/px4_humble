#include <double_integrator_test/double_integrator.hpp>

// PREPROCESSOR DIRECTIVES
// Using Statements
using std::placeholders::_1;

// CLASSES IMPLEMENTATION
DoubleIntegratorGovernor::DoubleIntegratorGovernor() : Node("Governor") {
	// PARAMETERS
	// Get Parameters
	// Set Parameters

	// VARIABLES
	reference_position = Eigen::Vector3d(0.0, 0.0, 0.0);
	reference_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
	reference_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

	drone_position = Eigen::Vector3d(0.0, 0.0, 0.0);

	takeoff_altitude = 1.0;
	PD_position_gain = 2.5;
	PD_velocity_gain = 0.5;
	setpoint_position = Eigen::Vector3d(0.0, 0.0, 0.0);
	setpoint_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
	setpoint_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

	dynamics_timer_frequency_ms = 50;
	state_machine_timer_frequency_ms = 100;
	

	agent_state = INIT;
	transition_takeoff = false;
	transition_takeoff_reached = false;

	// Debug
	debug_time = 0.0;
	euler_integration_step = 0.001;

	position_debug = Eigen::Vector3d::Zero();
	velocity_debug = Eigen::Vector3d::Zero();
	acceleration_debug = Eigen::Vector3d::Zero();

	std::shared_ptr<BezierSegment> p_segment_takeoff;
	std::shared_ptr<BezierSegment> p_segment_1;
	std::shared_ptr<CircleSegment> p_segment_2;
	std::shared_ptr<SpiralSegment> p_segment_3;
	std::shared_ptr<BezierSegment> p_segment_4;
	std::shared_ptr<BezierSegment> p_segment_land;

	double time_takeoff = 0.2;
	double time_segment_1 = 0.5;
	double time_segment_2 = 1.0;
	double time_segment_3 = 1.0;
	double time_segment_4 = 0.5;
	double time_land = 0.5;
	total_time = time_takeoff + time_segment_1 + time_segment_2 + time_segment_3 + time_segment_4 + time_land;

	// Takeoff
	auto control_points_takeoff = std::vector<Eigen::Vector3d>(8);
	control_points_takeoff[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
	control_points_takeoff[1] = Eigen::Vector3d(0.0, 0.0, 0.0);
	control_points_takeoff[2] = Eigen::Vector3d(0.0, 0.0, 0.0);
	control_points_takeoff[3] = Eigen::Vector3d(0.0, 0.0, 0.5);
	control_points_takeoff[4] = Eigen::Vector3d(0.0, 0.0, 0.5);
	control_points_takeoff[5] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_takeoff[6] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_takeoff[7] = Eigen::Vector3d(0.0, 0.0, 1.0);
	p_segment_takeoff = std::make_shared<BezierSegment>(time_takeoff, 7, control_points_takeoff);
	
	trajectory_debug.append_segment(p_segment_takeoff);

	// Segment 1
	auto control_points_segment_1 = std::vector<Eigen::Vector3d>(8);
	control_points_segment_1[0] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_1[1] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_1[2] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_1[3] = Eigen::Vector3d(2.5, 1.0, 1.5);
	control_points_segment_1[4] = Eigen::Vector3d(2.5, 1.0, 1.5);
	control_points_segment_1[5] = Eigen::Vector3d(5.0, 0.0, 1.0);
	control_points_segment_1[6] = Eigen::Vector3d(5.0, 0.0, 1.0);
	control_points_segment_1[7] = Eigen::Vector3d(5.0, 0.0, 1.0);
	p_segment_1 = std::make_shared<BezierSegment>(time_segment_1, 7, control_points_segment_1);
	
	trajectory_debug.append_segment(p_segment_1);
	
	// Segment 2
	Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.0, 1.0);
	double radius = 5.0;
	p_segment_2 = std::make_shared<CircleSegment>(time_segment_2, radius, center, 2.0 * M_PI / time_segment_2); 
	auto control_points_parameterization = std::vector<double>(8);
	control_points_parameterization[0] = 0.0;
	control_points_parameterization[1] = 0.0;
	control_points_parameterization[2] = 0.0;
	control_points_parameterization[3] = 0.3;
	control_points_parameterization[4] = 0.7;
	control_points_parameterization[5] = 1.0;
	control_points_parameterization[6] = 1.0;
	control_points_parameterization[7] = 1.0;
	auto circle_parameterization = std::make_shared<BezierParameterization>(7, control_points_parameterization, time_segment_2);
	p_segment_2->set_parameterization(circle_parameterization);

	trajectory_debug.append_segment(p_segment_2);

	// Segment 3
	p_segment_3 = std::make_shared<SpiralSegment>(time_segment_3, 1.0, radius, center, 2.0 * M_PI / time_segment_3, false);
	
	trajectory_debug.append_segment(p_segment_3);

	// Segment 4
	auto control_points_segment_4 = std::vector<Eigen::Vector3d>(8);
	control_points_segment_4[0] = Eigen::Vector3d(5.0, 0.0, 2.0);
	control_points_segment_4[1] = Eigen::Vector3d(5.0, 0.0, 2.0);
	control_points_segment_4[2] = Eigen::Vector3d(5.0, 0.0, 2.0);
	control_points_segment_4[3] = Eigen::Vector3d(2.5, 0.0, 1.5);
	control_points_segment_4[4] = Eigen::Vector3d(2.5, 0.0, 1.5);
	control_points_segment_4[5] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_4[6] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_4[7] = Eigen::Vector3d(0.0, 0.0, 1.0);
	p_segment_4 = std::make_shared<BezierSegment>(time_segment_4, 7, control_points_segment_4);
	
	trajectory_debug.append_segment(p_segment_4);
	
	// Segment land
	auto control_points_land = std::vector<Eigen::Vector3d>(8);
	control_points_land[0] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_land[1] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_land[2] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_land[3] = Eigen::Vector3d(0.0, 0.0, 0.5);
	control_points_land[4] = Eigen::Vector3d(0.0, 0.0, 0.5);
	control_points_land[5] = Eigen::Vector3d(0.0, 0.0, 0.0);
	control_points_land[6] = Eigen::Vector3d(0.0, 0.0, 0.0);
	control_points_land[7] = Eigen::Vector3d(0.0, 0.0, 0.0);
	p_segment_land = std::make_shared<BezierSegment>(time_land, 7, control_points_land);
	
	trajectory_debug.append_segment(p_segment_land);
	
	// QoS
	rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos_sensor_data = rclcpp::QoS(
				rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1), 
				qos_profile_sensor_data
			);

	rclcpp::QoS qos_reliable_transientLocal(1);
	qos_reliable_transientLocal.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	qos_reliable_transientLocal.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
	qos_reliable_transientLocal.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

	// TIMERS
	dynamics_timer = this->create_wall_timer(
				std::chrono::milliseconds(dynamics_timer_frequency_ms),
				std::bind(&DoubleIntegratorGovernor::dynamicsCallback, this)
			);
	
	state_machine_timer = this->create_wall_timer(
				std::chrono::milliseconds(state_machine_timer_frequency_ms),
				std::bind(&DoubleIntegratorGovernor::debugCallback, this)
			);

	// SUBSCRIPTIONS
	odometry_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
				"/fmu/out/vehicle_odometry", qos_sensor_data,
				std::bind(&DoubleIntegratorGovernor::odometryCallback, this, _1)
			);
	
	action_subscriber = this->create_subscription<std_msgs::msg::Empty>(
				"/action", qos_sensor_data,
				std::bind(&DoubleIntegratorGovernor::actionCallback, this, _1)
			);

	// PUBLISHERS
	setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
				"/fmu/in/trajectory_setpoint", qos_sensor_data
			);

	offboard_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
				"/fmu/in/offboard_control_mode", qos_sensor_data
			);
	
	trajectory_visualizer_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
				"/trajectory_markers", qos_sensor_data
			);

	setpoint_visualizer_publisher = this->create_publisher<visualization_msgs::msg::Marker>(
				"/setpoint_marker", qos_sensor_data
			);
	// TF
	tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this); 	
}

DoubleIntegratorGovernor::~DoubleIntegratorGovernor() {}

void DoubleIntegratorGovernor::debugCallback() {
	// Publish TF
	geometry_msgs::msg::TransformStamped t_map_to_odom;
	t_map_to_odom.header.stamp = this->get_clock()->now();
	t_map_to_odom.header.frame_id = "map";
	t_map_to_odom.child_frame_id = "odom";
	t_map_to_odom.transform.translation.x = 0.0;
	t_map_to_odom.transform.translation.y = 0.0;
	t_map_to_odom.transform.translation.z = 0.0;
	t_map_to_odom.transform.rotation.w = 1.0;
	t_map_to_odom.transform.rotation.x = 0.0;
	t_map_to_odom.transform.rotation.y = 0.0;
	t_map_to_odom.transform.rotation.z = 0.0;

	tf2_broadcaster->sendTransform(t_map_to_odom);

	// Simulate integrator
	for (int i = 0; i < dynamics_timer_frequency_ms; i++) {
		float tmp = debug_time + double(i) * euler_integration_step;
		velocity_debug = trajectory_debug.evaluate_velocity(tmp);
		position_debug += velocity_debug * euler_integration_step;
	}

	// Prepare messages
 	debug_time += double(dynamics_timer_frequency_ms) * 0.001;
	if (debug_time > total_time) debug_time = 0.0;
	Eigen::Vector3d marker_position = trajectory_debug.evaluate_position(debug_time);

	// Prepare trajectory
	visualization_msgs::msg::Marker trajectory_markers;
	trajectory_markers.header.stamp = this->get_clock()->now();
	trajectory_markers.header.frame_id = "map";
	trajectory_markers.id = 0;
	trajectory_markers.type = visualization_msgs::msg::Marker::POINTS;
	trajectory_markers.action = visualization_msgs::msg::Marker::ADD;
	trajectory_markers.pose.position.x = 0.0;
	trajectory_markers.pose.position.y = 0.0;
	trajectory_markers.pose.position.z = 0.0;
	trajectory_markers.pose.orientation.w = 1.0;
	trajectory_markers.pose.orientation.x = 0.0;
	trajectory_markers.pose.orientation.y = 0.0;
	trajectory_markers.pose.orientation.z = 0.0;
	trajectory_markers.scale.x = 0.1;
	trajectory_markers.scale.y = 0.1;
	trajectory_markers.scale.z = 0.0;
	trajectory_markers.color.a = 1.0;
	trajectory_markers.color.r = 0.0;
	trajectory_markers.color.g = 0.0;
	trajectory_markers.color.b = 0.0;
	trajectory_markers.points = std::vector<geometry_msgs::msg::Point>(1000);
	for (double i = 0; i < 1000; i++){
		Eigen::Vector3d point = trajectory_debug.evaluate_position(total_time / 1000.0 * i);;
		trajectory_markers.points[i].x = point.x();
		trajectory_markers.points[i].y = point.y();
		trajectory_markers.points[i].z = point.z();
	}

	// Prepare setpoint 1
	visualization_msgs::msg::Marker setpoint_marker;
	setpoint_marker.header.stamp = this->get_clock()->now();
	setpoint_marker.header.frame_id = "map";
	setpoint_marker.id = 3;
	setpoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
	setpoint_marker.action = visualization_msgs::msg::Marker::ADD;
	setpoint_marker.pose.position.x = marker_position.x();
	setpoint_marker.pose.position.y = marker_position.y();
	setpoint_marker.pose.position.z = marker_position.z();
	setpoint_marker.pose.orientation.w = 1.0;
	setpoint_marker.pose.orientation.x = 0.0;
	setpoint_marker.pose.orientation.y = 0.0;
	setpoint_marker.pose.orientation.z = 0.0;
	setpoint_marker.scale.x = 0.5;
	setpoint_marker.scale.y = 0.5;
	setpoint_marker.scale.z = 0.5;
	setpoint_marker.color.a = 0.5;
	setpoint_marker.color.r = 1.0;
	setpoint_marker.color.g = 0.0;
	setpoint_marker.color.b = 1.0;

	// Prepare setpoint 2
	visualization_msgs::msg::Marker setpoint_marker_2;
	setpoint_marker_2.header.stamp = this->get_clock()->now();
	setpoint_marker_2.header.frame_id = "map";
	setpoint_marker_2.id = 4;
	setpoint_marker_2.type = visualization_msgs::msg::Marker::SPHERE;
	setpoint_marker_2.action = visualization_msgs::msg::Marker::ADD;
	setpoint_marker_2.pose.position.x = position_debug.x();
	setpoint_marker_2.pose.position.y = position_debug.y();
	setpoint_marker_2.pose.position.z = position_debug.z();
	setpoint_marker_2.pose.orientation.w = 1.0;
	setpoint_marker_2.pose.orientation.x = 0.0;
	setpoint_marker_2.pose.orientation.y = 0.0;
	setpoint_marker_2.pose.orientation.z = 0.0;
	setpoint_marker_2.scale.x = 0.3;
	setpoint_marker_2.scale.y = 0.3;
	setpoint_marker_2.scale.z = 0.3;
	setpoint_marker_2.color.a = 0.3;
	setpoint_marker_2.color.r = 0.0;
	setpoint_marker_2.color.g = 1.0;
	setpoint_marker_2.color.b = 0.0;

	// Send messages
	trajectory_visualizer_publisher->publish(trajectory_markers);
	setpoint_visualizer_publisher->publish(setpoint_marker);
	setpoint_visualizer_publisher->publish(setpoint_marker_2);
}

void DoubleIntegratorGovernor::stateMachine() {
	switch(agent_state) {
		case INIT:
			RCLCPP_INFO(this->get_logger(), "[INIT] Waiting for start");
			if (transition_takeoff) {
				// Load setpoint data
				setpoint_position.x() = drone_position.x();
				setpoint_position.y() = drone_position.y();
				setpoint_position.z() = -drone_position.z() + takeoff_altitude;
				setpoint_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
				setpoint_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
	
				agent_state = TAKEOFF;
			}
			break;

		case TAKEOFF:
			RCLCPP_INFO(this->get_logger(), "[TAKEOFF] Reaching takeoff altitude");
			if (transition_takeoff_reached) {
				RCLCPP_INFO(this->get_logger(), "[TAKEOFF] Reached takeoff altitude");
			}
			break;

		case HOVER:
			break;

		case CIRCLE:
			RCLCPP_INFO(this->get_logger(), "[CIRCLE] Circular motion");
			if (transition_takeoff_reached) {
				RCLCPP_INFO(this->get_logger(), "[CIRCLE] Circle motion done");
			}
			break;

		case SPIRAL_UP:
			break;

		case SPIRAL_DOWN:
			break;

		case LAND:
			break;
	}
}

void DoubleIntegratorGovernor::actionCallback(const std_msgs::msg::Empty msg) {
	transition_takeoff = true;
}

void DoubleIntegratorGovernor::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
	// Read odometry and update agent position and acceleration
	drone_position.x() = msg->position[0];
	drone_position.y() = msg->position[1];
	drone_position.z() = msg->position[2];

	// agent_velocity.x() = msg.velocity[0];
	// agent_velocity.y() = msg.velocity[1];
	// agent_velocity.z() = msg.velocity[2];
}

void DoubleIntegratorGovernor::dynamicsCallback() {

	// Compute acceleration
	reference_acceleration = - PD_position_gain * ( reference_position - setpoint_position) 
		                       - PD_velocity_gain * ( reference_velocity - setpoint_velocity)
													 + setpoint_acceleration;

	reference_position += reference_velocity * double(dynamics_timer_frequency_ms) * 0.001;
	reference_velocity += reference_acceleration * double(dynamics_timer_frequency_ms) * 0.001;
	
	// Load and publish Offboard and Setpoint messages
	px4_msgs::msg::OffboardControlMode offboard_msg;
	px4_msgs::msg::TrajectorySetpoint setpoint_msg;

	// I should try also the acceleration only control
	offboard_msg.position = true;

	setpoint_msg.position[0] = reference_position.x();
	setpoint_msg.position[1] = reference_position.y();
	setpoint_msg.position[2] = -reference_position.z();

	setpoint_msg.velocity[0] = reference_velocity.x();
	setpoint_msg.velocity[1] = reference_velocity.y();
	setpoint_msg.velocity[2] = -reference_velocity.z();

	setpoint_msg.acceleration[0] = reference_acceleration.x();
	setpoint_msg.acceleration[1] = reference_acceleration.y();
	setpoint_msg.acceleration[2] = -reference_acceleration.z();

	offboard_publisher->publish(offboard_msg);
	setpoint_publisher->publish(setpoint_msg);
}

// MAIN
int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DoubleIntegratorGovernor>());
	rclcpp::shutdown();
	return 0;
}
