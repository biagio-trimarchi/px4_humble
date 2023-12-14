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
	std::vector<float> control_points_param = {0.0, 0.0, 0.0, 0.5, 0.5, 1.0, 1.0, 1.0};
	auto param_test = std::make_shared<BezierParameterization>(7, control_points_param, 10.0); 
	Eigen::Vector3d center(0.0, 0.0, 0.0);

	std::vector<Eigen::Vector3d> control_points = std::vector<Eigen::Vector3d>(8);
	control_points[0] = Eigen::Vector3d(0.0, 0.0, 0.0);
	control_points[1] = Eigen::Vector3d(1.0, 0.0, 0.0);
	control_points[2] = Eigen::Vector3d(2.0, 1.0, 1.0);
	control_points[3] = Eigen::Vector3d(1.0, 2.0, 2.0);
	control_points[4] = Eigen::Vector3d(0.0, 2.0, -2.0);
	control_points[5] = Eigen::Vector3d(-1.0, 2.0, -2.0);
	control_points[6] = Eigen::Vector3d(-1.0, 3.0, 1.0);
	control_points[7] = Eigen::Vector3d(0.0, 3.0, 0.0);

	trajectory_debug_1 = BezierSegment(10.0, 7, control_points);  
	trajectory_debug_2 = BezierSegment(10.0, 7, control_points);  
	trajectory_debug_2.set_parameterization(param_test);

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

	// Prepare messages
 	debug_time += double(dynamics_timer_frequency_ms) * 0.001;
	if (debug_time > 10.0) debug_time = 0.0;
	Eigen::Vector3d marker_position_1 = trajectory_debug_1.get_position(debug_time);
	Eigen::Vector3d marker_position_2 = trajectory_debug_2.get_position(debug_time);

	// Prepare trajectory
	visualization_msgs::msg::Marker trajectory_markers1;
	trajectory_markers1.header.stamp = this->get_clock()->now();
	trajectory_markers1.header.frame_id = "map";
	trajectory_markers1.id = 0;
	trajectory_markers1.type = visualization_msgs::msg::Marker::POINTS;
	trajectory_markers1.action = visualization_msgs::msg::Marker::ADD;
	trajectory_markers1.pose.position.x = 0.0;
	trajectory_markers1.pose.position.y = 0.0;
	trajectory_markers1.pose.position.z = 0.0;
	trajectory_markers1.pose.orientation.w = 1.0;
	trajectory_markers1.pose.orientation.x = 0.0;
	trajectory_markers1.pose.orientation.y = 0.0;
	trajectory_markers1.pose.orientation.z = 0.0;
	trajectory_markers1.scale.x = 0.1;
	trajectory_markers1.scale.y = 0.1;
	trajectory_markers1.scale.z = 0.0;
	trajectory_markers1.color.a = 1.0;
	trajectory_markers1.color.r = 0.0;
	trajectory_markers1.color.g = 0.0;
	trajectory_markers1.color.b = 0.0;
	trajectory_markers1.points = std::vector<geometry_msgs::msg::Point>(30);
	for (double i = 0; i < 30; i++){
		Eigen::Vector3d point = trajectory_debug_1.get_position(10.0 / 30.0 * i);;
		trajectory_markers1.points[i].x = point.x();
		trajectory_markers1.points[i].y = point.y();
		trajectory_markers1.points[i].z = point.z();
	}

	// Prepare trajectory
	visualization_msgs::msg::Marker trajectory_markers2;
	trajectory_markers2.header.stamp = this->get_clock()->now();
	trajectory_markers2.header.frame_id = "map";
	trajectory_markers2.id = 1;
	trajectory_markers2.type = visualization_msgs::msg::Marker::POINTS;
	trajectory_markers2.action = visualization_msgs::msg::Marker::ADD;
	trajectory_markers2.pose.position.x = 0.0;
	trajectory_markers2.pose.position.y = 0.0;
	trajectory_markers2.pose.position.z = 0.0;
	trajectory_markers2.pose.orientation.w = 1.0;
	trajectory_markers2.pose.orientation.x = 0.0;
	trajectory_markers2.pose.orientation.y = 0.0;
	trajectory_markers2.pose.orientation.z = 0.0;
	trajectory_markers2.scale.x = 0.1;
	trajectory_markers2.scale.y = 0.1;
	trajectory_markers2.scale.z = 0.0;
	trajectory_markers2.color.a = 1.0;
	trajectory_markers2.color.r = 0.0;
	trajectory_markers2.color.g = 0.0;
	trajectory_markers2.color.b = 0.0;
	trajectory_markers2.points = std::vector<geometry_msgs::msg::Point>(30);
	for (double i = 0; i < 30; i++){
		Eigen::Vector3d point = trajectory_debug_2.get_position(10.0 / 30.0 * i);;
		trajectory_markers2.points[i].x = point.x();
		trajectory_markers2.points[i].y = point.y();
		trajectory_markers2.points[i].z = point.z();
	}

	// Prepare setpoint
	visualization_msgs::msg::Marker setpoint_marker1;
	setpoint_marker1.header.stamp = this->get_clock()->now();
	setpoint_marker1.header.frame_id = "map";
	setpoint_marker1.id = 3;
	setpoint_marker1.type = visualization_msgs::msg::Marker::SPHERE;
	setpoint_marker1.action = visualization_msgs::msg::Marker::ADD;
	setpoint_marker1.pose.position.x = marker_position_1.x();
	setpoint_marker1.pose.position.y = marker_position_1.y();
	setpoint_marker1.pose.position.z = marker_position_1.z();
	setpoint_marker1.pose.orientation.w = 1.0;
	setpoint_marker1.pose.orientation.x = 0.0;
	setpoint_marker1.pose.orientation.y = 0.0;
	setpoint_marker1.pose.orientation.z = 0.0;
	setpoint_marker1.scale.x = 1.0;
	setpoint_marker1.scale.y = 1.0;
	setpoint_marker1.scale.z = 1.0;
	setpoint_marker1.color.a = 0.5;
	setpoint_marker1.color.r = 1.0;
	setpoint_marker1.color.g = 0.0;
	setpoint_marker1.color.b = 0.0;

	// Prepare setpoint
	visualization_msgs::msg::Marker setpoint_marker2;
	setpoint_marker2.header.stamp = this->get_clock()->now();
	setpoint_marker2.header.frame_id = "map";
	trajectory_markers1.id = 4;
	setpoint_marker2.type = visualization_msgs::msg::Marker::SPHERE;
	setpoint_marker2.action = visualization_msgs::msg::Marker::ADD;
	setpoint_marker2.pose.position.x = marker_position_2.x();
	setpoint_marker2.pose.position.y = marker_position_2.y();
	setpoint_marker2.pose.position.z = marker_position_2.z();
	setpoint_marker2.pose.orientation.w = 1.0;
	setpoint_marker2.pose.orientation.x = 0.0;
	setpoint_marker2.pose.orientation.y = 0.0;
	setpoint_marker2.pose.orientation.z = 0.0;
	setpoint_marker2.scale.x = 0.5;
	setpoint_marker2.scale.y = 0.5;
	setpoint_marker2.scale.z = 0.5;
	setpoint_marker2.color.a = 1.0;
	setpoint_marker2.color.r = 0.0;
	setpoint_marker2.color.g = 1.0;
	setpoint_marker2.color.b = 0.0;

	// Send messages
	trajectory_visualizer_publisher->publish(trajectory_markers1);
	trajectory_visualizer_publisher->publish(trajectory_markers2);
	setpoint_visualizer_publisher->publish(setpoint_marker1);
	setpoint_visualizer_publisher->publish(setpoint_marker2);
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
