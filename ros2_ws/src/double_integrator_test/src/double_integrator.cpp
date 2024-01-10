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
	drone_position = Eigen::Vector3d(0.0, 0.0, 0.0);
	drone_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);

	PD_position_gain = 5.5;
	PD_velocity_gain = 2.3;

	qpOASES_constraints_number = 1;
	qpOASES_solver = qpOASES::QProblem(3, qpOASES_constraints_number);
	qpOASES_options.printLevel = qpOASES::PL_NONE;
	qpOASES_solver.setOptions(qpOASES_options);
	qpOASES_H = Eigen::Matrix3d::Identity();
	qpOASES_A = Eigen::MatrixXd::Zero(qpOASES_constraints_number, 3);
	qpOASES_lbA = Eigen::VectorXd::Zero(qpOASES_constraints_number);
	qpOASES_ubA = 10.0 * Eigen::VectorXd::Ones(qpOASES_constraints_number);
	qpOASES_lb << -10.0, -10.0, -10.0;
	qpOASES_ub << 10.0, 10.0, 10.0;

	gp_lambda_whittle = 5.0;
	gp_resolution = 0.2;
	gp_error_variance = 0.01;
	log_gpis = LogGPIS(gp_lambda_whittle, gp_resolution, gp_error_variance);
	is_log_gpis_trained = false;
	bf_classK_gain_1 = 2.0;
	bf_classK_gain_1 = 0.5;
	bf_gain_lie_0_kh = bf_classK_gain_1 * bf_classK_gain_2;
	bf_gain_lie_1_kh = bf_classK_gain_1 + bf_classK_gain_2;
	bf_safe_margin = 0.1;

	addGroundSphereCylinder();

	takeoff_altitude = 1.0;
	setpoint_position = Eigen::Vector3d(0.0, 0.0, 0.0);
	setpoint_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
	setpoint_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
	trajectory_time = 0.0;

	debug_timer_frequency_ms = 10;
	dynamics_timer_frequency_ms = 10;
	state_machine_timer_frequency_ms = 100;

	agent_state = INIT;
	message_received = false;
	transition_takeoff_reached = false;
	is_drone_armed = false;
	is_drone_offboard = false;
	follow_trajectory = false;

	std::shared_ptr<BezierSegment> p_segment_takeoff;
	std::shared_ptr<BezierSegment> p_segment_1;
	std::shared_ptr<CircleSegment> p_segment_2;
	std::shared_ptr<SpiralSegment> p_segment_3;
	std::shared_ptr<BezierSegment> p_segment_4;
	std::shared_ptr<BezierSegment> p_segment_land;

	double time_takeoff = 20.0;
	double time_segment_1 = 20.0;
	double time_segment_2 = 20.0;
	double time_segment_3 = 20.0;
	double time_segment_4 = 40.0;
	double time_land = 10.0;
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
	
	trajectory.append_segment(p_segment_takeoff);

	// Segment 1
	double radius = 3.0;
	auto control_points_segment_1 = std::vector<Eigen::Vector3d>(8);
	control_points_segment_1[0] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_1[1] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_1[2] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_1[3] = Eigen::Vector3d(radius/2.0, 1.0, 1.5);
	control_points_segment_1[4] = Eigen::Vector3d(radius/2.0, 1.0, 1.5);
	control_points_segment_1[5] = Eigen::Vector3d(radius, 0.0, 1.0);
	control_points_segment_1[6] = Eigen::Vector3d(radius, 0.0, 1.0);
	control_points_segment_1[7] = Eigen::Vector3d(radius, 0.0, 1.0);
	p_segment_1 = std::make_shared<BezierSegment>(time_segment_1, 7, control_points_segment_1);
	
	trajectory.append_segment(p_segment_1);
	
	// Segment 2
	Eigen::Vector3d center = Eigen::Vector3d(0.0, 0.0, 1.0);
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

	trajectory.append_segment(p_segment_2);

	// Segment 3
	p_segment_3 = std::make_shared<SpiralSegment>(time_segment_3, 1.0, radius, center, 2.0 * M_PI / time_segment_3, false);
	p_segment_3->set_parameterization(circle_parameterization);
	
	trajectory.append_segment(p_segment_3);
	
	// Segment 4
	auto control_points_segment_4 = std::vector<Eigen::Vector3d>(8);
	control_points_segment_4[0] = Eigen::Vector3d(radius, 0.0, 2.0);
	control_points_segment_4[1] = Eigen::Vector3d(radius, 0.0, 2.0);
	control_points_segment_4[2] = Eigen::Vector3d(radius, 0.0, 2.0);
	control_points_segment_4[3] = Eigen::Vector3d(radius/2.0, 0.0, 1.5);
	control_points_segment_4[4] = Eigen::Vector3d(radius/2.0, 0.0, 1.5);
	control_points_segment_4[5] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_4[6] = Eigen::Vector3d(0.0, 0.0, 1.0);
	control_points_segment_4[7] = Eigen::Vector3d(0.0, 0.0, 1.0);
	p_segment_4 = std::make_shared<BezierSegment>(time_segment_4, 7, control_points_segment_4);
	
	trajectory.append_segment(p_segment_4);
	
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
	
	trajectory.append_segment(p_segment_land);
	
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
	odometry_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
				"/fmu/out/vehicle_odometry", qos_sensor_data,
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
	
	// TF
	tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this); 	
}

DoubleIntegratorGovernor::~DoubleIntegratorGovernor() {}

void DoubleIntegratorGovernor::debugCallback() {
	visualization_msgs::msg::Marker msg;
	msg.header.frame_id = "map";
	msg.header.stamp = this->get_clock()->now();
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
	msg.color.r = 1.0;
	msg.color.g = 0.0;
	msg.color.b = 0.0;
	msg.color.a = 1.0;
	setpoint_visualizer_publisher->publish(msg);
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
			if (message_received) {
				message_received = false;
				setModeOffboard();
				arm();
			}
			if (is_drone_armed && is_drone_offboard) {
				agent_state = ARMED;
			}
			break;

		case ARMED:
			RCLCPP_INFO(this->get_logger(), "[ARMED] Waiting for takeoff");
			if (message_received) {
				message_received = false;
				trajectory_time = 0.0;
				follow_trajectory = true;
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

		case FOLLOW_TRAJECTORY:
			RCLCPP_INFO(this->get_logger(), "[CIRCLE] Circular motion");
			if (transition_takeoff_reached) {
				RCLCPP_INFO(this->get_logger(), "[CIRCLE] Circle motion done");
			}
			break;

		case LAND:
			break;
	}
}

void DoubleIntegratorGovernor::actionCallback(const std_msgs::msg::Empty msg) {
	message_received = true;
}

void DoubleIntegratorGovernor::odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
	// Read odometry and update agent position and acceleration
	drone_position.x() = msg->position[0];
	drone_position.y() = -msg->position[1];
	drone_position.z() = -msg->position[2];

	drone_velocity.x() = msg->velocity[0];
	drone_velocity.y() = -msg->velocity[1];
	drone_velocity.z() = -msg->velocity[2];
}

void DoubleIntegratorGovernor::px4StatusCallback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
	is_drone_armed = msg->flag_armed;
	is_drone_offboard = msg->flag_control_offboard_enabled;
}

void DoubleIntegratorGovernor::controlBarrierFunction() {
	if (!is_log_gpis_trained)
		return;
	
	// Compute barrier functions needed derivatives
	double h = log_gpis.evaluate(drone_position) - bf_safe_margin;;
	Eigen::RowVector<double, 6> gradient_h;
	Eigen::Matrix<double, 6, 6> hessian_h;

	gradient_h.setZero();
	gradient_h.block(0, 0,  1, 3) = log_gpis.gradient(drone_position); 

	hessian_h.setZero();
	hessian_h.block(0, 0, 3, 3) = log_gpis.hessian(drone_position);

	// Compute lie derivatives
	Eigen::Vector<double, 6> integrator_state;
	Eigen::Matrix<double, 6, 6> integrator_dynamic_A;
	Eigen::Matrix<double, 6, 3> integrator_dynamic_B;

	integrator_state.block(0,0,3,1) = drone_position;
	integrator_state.block(3,0,3,1) = drone_velocity;

	integrator_dynamic_A.setZero();
	integrator_dynamic_A.block(0,3,3,3) = Eigen::Matrix3d::Identity();

	integrator_dynamic_B.setZero();
	integrator_dynamic_B.block(3,0,3,3) = Eigen::Matrix3d::Identity();

	double lie_f_h = gradient_h * integrator_dynamic_A * integrator_state;
	double lie_f2_h_1 = integrator_state.transpose() * integrator_dynamic_A.transpose() * hessian_h * integrator_dynamic_A * integrator_state; 
	double lie_f2_h_2 = gradient_h * integrator_dynamic_A * integrator_dynamic_A * integrator_state;
	double lie_f2_h = lie_f2_h_1 + lie_f2_h_2;

	Eigen::RowVector3d lie_gf_h = integrator_state.transpose() * integrator_dynamic_A.transpose() * hessian_h * integrator_dynamic_B +
	                           gradient_h * integrator_dynamic_A * integrator_dynamic_B;

	qpOASES_nWSR = 10;
	qpOASES_g = -setpoint_acceleration;
	qpOASES_A.block(0, 0, 1, 3) = lie_gf_h;
	qpOASES_lbA(0) = - lie_f2_h - bf_gain_lie_1_kh * lie_f_h - bf_gain_lie_0_kh * h;

	qpOASES_solver.init(qpOASES_H.data(), qpOASES_g.data(),  qpOASES_A.data(),
	                    qpOASES_lb.data(), qpOASES_ub.data(), qpOASES_lbA.data(),
	                    qpOASES_ubA.data(), qpOASES_nWSR);

	qpOASES_solver.getPrimalSolution(setpoint_acceleration.data());
}

void DoubleIntegratorGovernor::dynamicsCallback() {
	setpoint_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);
	if (follow_trajectory) {
		setpoint_position = trajectory.evaluate_position(trajectory_time);
		setpoint_velocity = trajectory.evaluate_velocity(trajectory_time);
		setpoint_acceleration = trajectory.evaluate_acceleration(trajectory_time); 
		trajectory_time += double(dynamics_timer_frequency_ms) * 0.001;
		if (trajectory_time > total_time) trajectory_time = total_time;
	}

	// Compute acceleration
	setpoint_acceleration += - PD_position_gain * ( drone_position - setpoint_position) 
		                        - PD_velocity_gain * ( drone_velocity - setpoint_velocity); 
	controlBarrierFunction();
	
	// Load and publish Offboard and Setpoint messages
	px4_msgs::msg::OffboardControlMode offboard_msg;
	px4_msgs::msg::TrajectorySetpoint setpoint_msg;

	// offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	// setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

	// I should try also the acceleration only control
	offboard_msg.position = false;
	offboard_msg.velocity = false;
	offboard_msg.acceleration = true;

	setpoint_msg.position[0] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.position[1] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.position[2] = std::numeric_limits<double>::quiet_NaN();

	setpoint_msg.velocity[0] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.velocity[1] = std::numeric_limits<double>::quiet_NaN();
	setpoint_msg.velocity[2] = std::numeric_limits<double>::quiet_NaN();

	// setpoint_msg.position[0] = setpoint_position.x();
	// setpoint_msg.position[1] = -setpoint_position.y();
	// setpoint_msg.position[2] = -setpoint_position.z();

	// setpoint_msg.velocity[0] = setpoint_velocity.x();
	// setpoint_msg.velocity[1] = -setpoint_velocity.y();
	// setpoint_msg.velocity[2] = -setpoint_velocity.z();

	setpoint_msg.acceleration[0] = setpoint_acceleration.x();
	setpoint_msg.acceleration[1] = -setpoint_acceleration.y();
	setpoint_msg.acceleration[2] = -setpoint_acceleration.z();

	offboard_publisher->publish(offboard_msg);
	setpoint_publisher->publish(setpoint_msg);
}

void DoubleIntegratorGovernor::addGroundSphereCylinder() {
		// Add ground
		for (double x = -5.0; x < 5.1; x += 0.5) {
			for (double y = -5.0; y < 5.1; y += 0.5) {
				Eigen::Vector3d point(x, y, -0.5);
				log_gpis.add_sample(point);
			}
		}

		// Add Sphere
		Eigen::Vector3d sphere_center(2.0, 2.0, 3.0);
		double sphere_radius = 0.3;
		for (double psi = 0.0; psi < M_PI+0.1; psi += 0.2){
			for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
				Eigen::Vector3d point(sin(psi) * cos(theta),
				                      sin(psi) * sin(theta),
				                      cos(psi)
				                     );
				point = sphere_center + sphere_radius * point;
				log_gpis.add_sample(point);
			}
		}

		Eigen::Vector3d cylinder_center_ground(-1.0, -3.0, 0.0);
		double cylinder_radius = 0.5;
		for (double z = 0.0; z < 2.1; z += 0.2){
			for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
				Eigen::Vector3d point(cylinder_radius * cos(theta), 
				                      cylinder_radius * sin(theta), 
				                      z);
				point = cylinder_center_ground + cylinder_radius * point;
				log_gpis.add_sample(point);
			}
		}

		RCLCPP_INFO(this->get_logger(), "Training...");
		log_gpis.train();
		is_log_gpis_trained = true;
		RCLCPP_INFO(this->get_logger(), "Done!");
}

// MAIN
int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DoubleIntegratorGovernor>());
	rclcpp::shutdown();
	return 0;
}
