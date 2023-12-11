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

	std::vector<float> control_points;
	control_points.push_back(0.0);
	control_points.push_back(2.0);
	control_points.push_back(5.0);
	test_bez = BezierParameterization(2, control_points, 20.0);

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
	
	// TF
	
}

DoubleIntegratorGovernor::~DoubleIntegratorGovernor() {}

void DoubleIntegratorGovernor::debugCallback() {
	std::cout << "Identity: (0) " << test_param.evaluate_function(0.2) << std::endl;
	std::cout << "Bezier test: (0) " << test_bez.evaluate_function(20) << std::endl;
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
