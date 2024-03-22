#include <simulation_manager/double_integrator_test.hpp>

Test::Test() : Node("Test") {
	// Variables
	reference_position = Eigen::Vector3d(0.0, 0.0, 0.0);
	reference_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
	reference_acceleration = Eigen::Vector3d(0.0, 0.0, 0.0);

	setpoint_position = Eigen::Vector3d(8.0, 7.0, 3.0);
	setpoint_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);

	delta_t = 0.03;
	total_simulation_time = 50;
	controller_P_gain = 0.2;
	controller_D_gain = 1.0;

	qpOASES_constraints_number = 1;
	qpOASES_solver = qpOASES::QProblem(3, qpOASES_constraints_number);
	qpOASES_options.printLevel = qpOASES::PL_NONE;
	qpOASES_solver.setOptions(qpOASES_options);
	qpOASES_H = Eigen::Matrix3d::Identity();
	qpOASES_A = Eigen::MatrixXd::Zero(qpOASES_constraints_number, 3);
	qpOASES_lbA = Eigen::VectorXd::Zero(qpOASES_constraints_number);
	qpOASES_ubA = 5.0 * Eigen::VectorXd::Ones(qpOASES_constraints_number);
	qpOASES_lb << -5.0, -5.0, -5.0;
	qpOASES_ub << 5.0, 5.0, 5.0;

	bf_classK_gain_1 = 1.10;
	bf_classK_gain_2 = 1.10;
	bf_gain_lie_0_kh = bf_classK_gain_1 * bf_classK_gain_2;
	bf_gain_lie_1_kh = bf_classK_gain_1 + bf_classK_gain_2;
	bf_safe_margin = 0.2;

	gp_lambda_whittle = 7.50;
	gp_resolution = 0.1;
	gp_error_variance = 0.01;
	log_gpis = LogGPIS(gp_lambda_whittle, gp_resolution, gp_error_variance);

	trajectory_msg = visualization_msgs::msg::Marker();
	debug_msg = visualization_msgs::msg::Marker();

	addObstacles();
	
	// Timers
	timer_visualization = this->create_wall_timer(
		std::chrono::milliseconds(1000),
		std::bind(&Test::debugCallback, this)
	);
	
	// QoS
	rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos_sensor_data = rclcpp::QoS(
	  rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
	  qos_profile_sensor_data
	);

	rclcpp::QoS qos_reliable_transient_local(1);
	qos_reliable_transient_local.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
	qos_reliable_transient_local.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
	qos_reliable_transient_local.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
	
	// Publishers
	publisher_trajectory = this->create_publisher<visualization_msgs::msg::Marker>(
	  "/trajectory", qos_reliable_transient_local
	);

	publisher_obstacles = this->create_publisher<visualization_msgs::msg::Marker>(
	  "/obstacles", qos_reliable_transient_local
	);

	simulate();
}

Test::~Test() {}

void Test::addObstacles() {
	// Sphere 1
	Eigen::Vector3d sphere_center(6.0, -5.0, 1.0);
		double sphere_radius = 3.0;
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
		
		// Add Cylinder 1
		Eigen::Vector3d cylinder_center_ground(5.0, 4.0, 0.0);
		double cylinder_radius = 2.00;
		for (double z = 0.0; z < 5.1; z += 0.2){
			for (double theta = 0.0; theta < 2*M_PI; theta += 0.2){
				Eigen::Vector3d point(cylinder_radius * cos(theta), 
				                      cylinder_radius * sin(theta), 
				                      z);
				point = cylinder_center_ground + point;
				log_gpis.add_sample(point);
			}
		}

		RCLCPP_INFO(this->get_logger(), "Training...");
		log_gpis.train();
		RCLCPP_INFO(this->get_logger(), "Training done");
		RCLCPP_INFO(this->get_logger(), "Scanning...");
		scanRegion();
		RCLCPP_INFO(this->get_logger(), "Scanning done");
}

void Test::scanRegion() {
	double scan_resolution = 0.3;
	for (double x = -0.0; x < 8.1; x+=scan_resolution){
		for (double y = -8.0; y < 8.1; y+=scan_resolution){
			for (double z = -2.0; z < 6.1; z+=scan_resolution){
				Eigen::Vector3d scan_point(x, y, z);
				double value = log_gpis.evaluate(scan_point) - bf_safe_margin;
				if (value < 0.0 && value > -10.0) {
					geometry_msgs::msg::Point point;
					point.x = x;
					point.y = y;
					point.z = z;
					std_msgs::msg::ColorRGBA color;
					color.r = 0.0;
					color.g = value / 5.0;
					color.b = 1.0 - value / 5.0;
					color.a = 1.0;
					debug_msg.points.push_back(point);
					debug_msg.colors.push_back(color);
				}
			}
		}
	}
}

void Test::controller() {
	reference_acceleration = - controller_P_gain * (reference_position - setpoint_position)
	                         - controller_D_gain * (reference_velocity - setpoint_velocity);
}

void Test::barrierFunction() {	
	// Compute barrier functions needed derivatives
	double h = log_gpis.evaluate(reference_position) - bf_safe_margin;
	Eigen::RowVector<double, 6> gradient_h;
	Eigen::Matrix<double, 6, 6> hessian_h;

	gradient_h.setZero();
	gradient_h.block(0, 0,  1, 3) = log_gpis.gradient(reference_position); 

	hessian_h.setZero();
	hessian_h.block(0, 0, 3, 3) = log_gpis.hessian(reference_position);

	// Compute lie derivatives
	Eigen::Vector<double, 6> integrator_state;
	Eigen::Matrix<double, 6, 6> integrator_dynamic_A;
	Eigen::Matrix<double, 6, 3> integrator_dynamic_B;

	integrator_state.block(0,0,3,1) = reference_position;
	integrator_state.block(3,0,3,1) = reference_velocity;

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
	qpOASES_g = -reference_acceleration;
	qpOASES_A.block(0, 0, 1, 3) = lie_gf_h;
	qpOASES_lbA(0) = - lie_f2_h - bf_gain_lie_1_kh * lie_f_h - bf_gain_lie_0_kh * h;

	qpOASES_solver.init(qpOASES_H.data(), qpOASES_g.data(),  qpOASES_A.data(),
	                    qpOASES_lb.data(), qpOASES_ub.data(), qpOASES_lbA.data(),
	                    qpOASES_ubA.data(), qpOASES_nWSR);

	qpOASES_solver.getPrimalSolution(reference_acceleration.data());
}

void Test::dynamics() {
	controller();
	barrierFunction();
	reference_position += reference_velocity * delta_t;
	reference_velocity += reference_acceleration * delta_t;
}

void Test::simulate() {
	RCLCPP_INFO(this->get_logger(), "Simulating...");
	for (double time = 0.0; time < total_simulation_time; time+=delta_t) {
		dynamics();

		geometry_msgs::msg::Point point;
		point.x = reference_position.x();
		point.y = reference_position.y();
		point.z = reference_position.z();
		trajectory_msg.points.push_back(point);
	}
}

void Test::debugCallback() {
	RCLCPP_INFO(this->get_logger(), "Preparing trajectory message...");

	trajectory_msg.header.frame_id = "map";
	trajectory_msg.header.stamp = this->get_clock()->now();
	trajectory_msg.type = visualization_msgs::msg::Marker::POINTS;
	trajectory_msg.action = visualization_msgs::msg::Marker::ADD;
	trajectory_msg.pose.position.x = 0.0;
	trajectory_msg.pose.position.y = 0.0;
	trajectory_msg.pose.position.z = 0.0;
	trajectory_msg.pose.orientation.w = 1.0;
	trajectory_msg.pose.orientation.x = 0.0;
	trajectory_msg.pose.orientation.y = 0.0;
	trajectory_msg.pose.orientation.z = 0.0;
	trajectory_msg.scale.x = 0.1;
	trajectory_msg.scale.y = 0.1;
	trajectory_msg.scale.z = 0.1;
	trajectory_msg.color.r = 0.0;
	trajectory_msg.color.g = 1.0;
	trajectory_msg.color.b = 0.0;
	trajectory_msg.color.a = 1.0;

	publisher_trajectory->publish(trajectory_msg);
	RCLCPP_INFO(this->get_logger(), "Message sent!");

	RCLCPP_INFO(this->get_logger(), "Preparing obstacle message...");

	debug_msg.header.frame_id = "map";
	debug_msg.header.stamp = this->get_clock()->now();
	debug_msg.type = visualization_msgs::msg::Marker::POINTS;
	debug_msg.action = visualization_msgs::msg::Marker::ADD;
	debug_msg.pose.position.x = 0.0;
	debug_msg.pose.position.y = 0.0;
	debug_msg.pose.position.z = 0.0;
	debug_msg.pose.orientation.w = 1.0;
	debug_msg.pose.orientation.x = 0.0;
	debug_msg.pose.orientation.y = 0.0;
	debug_msg.pose.orientation.z = 0.0;
	debug_msg.scale.x = 0.05;
	debug_msg.scale.y = 0.05;
	debug_msg.scale.z = 0.05;
	// debug_msg.color.r = 1.0;
	// debug_msg.color.g = 0.0;
	// debug_msg.color.b = 0.0;
	// debug_msg.color.a = 1.0;

	publisher_obstacles->publish(debug_msg);
	RCLCPP_INFO(this->get_logger(), "Message sent!");
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Test>());
	rclcpp::shutdown();
	return 0;
}
