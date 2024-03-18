#include <double_integrator_test/test_takeoff.hpp>

// PREPROCESSOR DIRECTIVES
// Using Statements
using std::placeholders::_1;

// CLASSES IMPLEMENTATION
DoubleIntegratorGovernor::DoubleIntegratorGovernor() : Node("Governor") {
	// PARAMETERS
	// Get Parameters
	// Set Parameters

	// VARIABLES
	drone_position = Eigen::Vector3d(0.0, 0.0, 0.0);
	drone_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
	drone_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
	
	reference_position = Eigen::Vector3d(0.0, 0.0, 0.0);
	q_reference_yaw = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
	reference_yaw = 0;

	takeoff_altitude = 1.0;

	debug_timer_frequency_ms = 10;
	dynamics_timer_frequency_ms = 10;
	state_machine_timer_frequency_ms = 100;

	agent_state = INIT;
	message_received = false;
	transition_setpoint_reached = false;
	is_drone_armed = false;
	is_drone_offboard = false;
	
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
	debug_timer = this->create_wall_timer(
				std::chrono::milliseconds(debug_timer_frequency_ms),
				std::bind(&DoubleIntegratorGovernor::debugCallback, this)
			);

	dynamics_timer = this->create_wall_timer(
				std::chrono::milliseconds(dynamics_timer_frequency_ms),
				std::bind(&DoubleIntegratorGovernor::dynamicsCallback, this)
			);
	
	state_machine_timer = this->create_wall_timer(
				std::chrono::milliseconds(state_machine_timer_frequency_ms),
				std::bind(&DoubleIntegratorGovernor::stateMachine, this)
			);

	// SUBSCRIPTIONS
	odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
				"/odometry", qos_sensor_data,
				std::bind(&DoubleIntegratorGovernor::odometryCallback, this, _1)
			);
	
	action_subscriber = this->create_subscription<std_msgs::msg::Empty>(
				"/action", qos_sensor_data,
				std::bind(&DoubleIntegratorGovernor::actionCallback, this, _1)
			);

	vehicle_state_subscriber = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
				"/fmu/out/vehicle_control_mode", qos_sensor_data,
				std::bind(&DoubleIntegratorGovernor::px4StatusCallback, this, _1)
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
				"/setpoint_marker", 10
			);

	vehicle_command_publisher = this->create_publisher<px4_msgs::msg::VehicleCommand>(
				"/fmu/in/vehicle_command", qos_sensor_data
			);

	RCLCPP_INFO(this->get_logger(), "Initialization complete");
}

DoubleIntegratorGovernor::~DoubleIntegratorGovernor() {}

void DoubleIntegratorGovernor::debugCallback() {
	visualization_msgs::msg::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = this->get_clock()->now();
	msg.id = 1;
	msg.type = visualization_msgs::msg::Marker::SPHERE;
	msg.action = visualization_msgs::msg::Marker::ADD;
	msg.pose.position.x = drone_position.x();
	msg.pose.position.y = drone_position.y();
	msg.pose.position.z = drone_position.z();
	msg.pose.orientation.w = 1.0;
	msg.pose.orientation.x = 0.0;
	msg.pose.orientation.y = 0.0;
	msg.pose.orientation.z = 0.0;
	msg.scale.x = 0.3;
	msg.scale.y = 0.3;
	msg.scale.z = 0.3;
	msg.color.r = 0.0;
	msg.color.g = 0.0;
	msg.color.b = 1.0;
	msg.color.a = 1.0;
	setpoint_visualizer_publisher->publish(msg);

	msg.header.frame_id = "map";
	msg.header.stamp = this->get_clock()->now();
	msg.id = 2;
	msg.type = visualization_msgs::msg::Marker::SPHERE;
	msg.action = visualization_msgs::msg::Marker::ADD;
	msg.pose.position.x = reference_position.x();
	msg.pose.position.y = reference_position.y();
	msg.pose.position.z = reference_position.z();
	msg.pose.orientation.w = 1.0;
	msg.pose.orientation.x = 0.0;
	msg.pose.orientation.y = 0.0;
	msg.pose.orientation.z = 0.0;
	msg.scale.x = 0.6;
	msg.scale.y = 0.6;
	msg.scale.z = 0.6;
	msg.color.r = 1.0;
	msg.color.g = 0.0;
	msg.color.b = 1.0;
	msg.color.a = 0.5;
	trajectory_visualizer_publisher->publish(msg);
}

void DoubleIntegratorGovernor::publishPX4Command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
	px4_msgs::msg::VehicleCommand msg;
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher->publish(msg);
}

void DoubleIntegratorGovernor::setModeOffboard() {
	publishPX4Command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0);
	RCLCPP_INFO(this->get_logger(), "--- Setting Offboard Mode ---");
}

void DoubleIntegratorGovernor::arm() {
	publishPX4Command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "--- Arming ---");
}

void DoubleIntegratorGovernor::disarm() {
	publishPX4Command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "--- Disarming ---");
}

void DoubleIntegratorGovernor::stateMachine() {
	switch(agent_state) {
		case INIT:
			RCLCPP_INFO(this->get_logger(), "[INIT] Waiting for start");
			if (is_drone_armed && is_drone_offboard) {
				agent_state = ARMED;
			}
			break;

		case ARMED:
			RCLCPP_INFO(this->get_logger(), "[ARMED] Waiting for takeoff");
			if (message_received) {
				message_received = false;
				agent_state = TAKEOFF;
				reference_position = Eigen::Vector3d(0.0, 0.0, takeoff_altitude);
			}
			break;

		case TAKEOFF:
			RCLCPP_INFO(this->get_logger(), "[TAKEOFF] Reaching takeoff altitude");
			if (transition_setpoint_reached) {
				transition_setpoint_reached = false;
				RCLCPP_INFO(this->get_logger(), "[TAKEOFF] Reached takeoff altitude");
				reference_position = Eigen::Vector3d(0.0, 0.0, takeoff_altitude);
				agent_state = HOVER;
			}
			break;

		case HOVER:
			RCLCPP_INFO(this->get_logger(), "[HOVER] Hovering in position");
			if (message_received) {
				message_received = false;
				agent_state = ROTATE_RIGHT;
				reference_position = Eigen::Vector3d(0.0, 0.0, takeoff_altitude);
				reference_yaw = -M_PI/2.0;
				q_reference_yaw = Eigen::Quaterniond(0.0, 0.0, 0.0, -1.0);
			}
			break;

		case ROTATE_RIGHT:
			RCLCPP_INFO(this->get_logger(), "[ROTATE RIGHT]: Rotating");
			if (transition_setpoint_reached) {
				RCLCPP_INFO(this->get_logger(), "[ROTATE RIGHT] Rotated");
				transition_setpoint_reached = false;
				reference_position = Eigen::Vector3d(0.0, -1.0, takeoff_altitude);
				agent_state = MOVE_RIGHT;
			}
			break;

		case MOVE_RIGHT:
			RCLCPP_INFO(this->get_logger(), "[MOVE RIGHT]: Moving");
			if (transition_setpoint_reached) {
				RCLCPP_INFO(this->get_logger(), "[MOVE RIGHT] Stop");
				transition_setpoint_reached = false;
				reference_position = Eigen::Vector3d(0.0, -1.0, 0.0);
				agent_state = LAND;
			}
			break;

		case LAND:
			RCLCPP_INFO(this->get_logger(), "[LAND] Landing");
			if (transition_setpoint_reached) {
				RCLCPP_INFO(this->get_logger(), "[LAND] Landed!");
				follow_trajectory = false;
				reference_position = Eigen::Vector3d(0.0, -1.0, 0.0);
				agent_state = INIT;
			}
			break;
	}
}

void DoubleIntegratorGovernor::actionCallback(const std_msgs::msg::Empty::SharedPtr msg) {
	message_received = true;
}

void DoubleIntegratorGovernor::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
	// Read odometry and update agent position and acceleration
	drone_position.x() = msg->pose.pose.position.x;
	drone_position.y() = msg->pose.pose.position.y;
	drone_position.z() = msg->pose.pose.position.z;

	drone_velocity.x() = msg->twist.twist.linear.x;
	drone_velocity.y() = msg->twist.twist.linear.y;
	drone_velocity.z() = msg->twist.twist.linear.z;

	drone_orientation.w() = msg->pose.pose.orientation.w;
	drone_orientation.x() = msg->pose.pose.orientation.x;
	drone_orientation.y() = msg->pose.pose.orientation.y;
	drone_orientation.z() = msg->pose.pose.orientation.z;
}

void DoubleIntegratorGovernor::px4StatusCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
	is_drone_armed = msg->flag_armed;
	is_drone_offboard = msg->flag_control_offboard_enabled;
}

void DoubleIntegratorGovernor::dynamicsCallback() {
	if ((drone_position - reference_position).norm() < 0.1 && std::abs(drone_orientation.angularDistance(q_reference_yaw)) < 0.01) {
		transition_setpoint_reached = true;
	}

	// Load and publish Offboard and Setpoint messages
	px4_msgs::msg::OffboardControlMode offboard_msg;
	px4_msgs::msg::TrajectorySetpoint setpoint_msg;

	offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	// I should try also the acceleration only control
	offboard_msg.position = true;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = false;

	setpoint_msg.position[0] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.position[1] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.position[2] = std::numeric_limits<double>::quiet_NaN();

	setpoint_msg.velocity[0] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.velocity[1] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.velocity[2] = std::numeric_limits<double>::quiet_NaN();

	setpoint_msg.acceleration[0] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.acceleration[1] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.acceleration[2] = std::numeric_limits<double>::quiet_NaN();

	setpoint_msg.position[0] = reference_position.y();
	setpoint_msg.position[1] = reference_position.x();
	setpoint_msg.position[2] = -reference_position.z();
	setpoint_msg.yaw = reference_yaw;

	// setpoint_msg.velocity[0] = reference_velocity.y();
	// setpoint_msg.velocity[1] = reference_velocity.x();
	// setpoint_msg.velocity[2] = -reference_velocity.z();

	// setpoint_msg.acceleration[0] = setpoint_acceleration.y();
	// setpoint_msg.acceleration[1] = setpoint_acceleration.x();
	// setpoint_msg.acceleration[2] = -setpoint_acceleration.z();

	offboard_publisher->publish(offboard_msg);
	setpoint_publisher->publish(setpoint_msg);
}

// MAIN
int main(int argc, char** argv){
	rclcpp::init(argc, argv);

	rclcpp::Node::SharedPtr node_governor = std::make_shared<DoubleIntegratorGovernor>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node_governor);
	executor.spin();

	rclcpp::shutdown();
	return 0;
}
