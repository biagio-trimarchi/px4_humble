// LIBRARIES
#include <double_integrator_test/model_state_publisher.hpp>

// CLASSES
GazeboModelStatePublisher::GazeboModelStatePublisher(std::string model_name) : Node("GazeboModelStatePublisher"){
	this->model_name = model_name;
	timer_read_model_state_frequency_ms = 10;
	
	// Qos
	rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos_sensor_data = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1), 
    qos_profile_sensor_data
  );

	// TIMERS
	timer_read_model_state = this->create_wall_timer(
    std::chrono::milliseconds(timer_read_model_state_frequency_ms),
    std::bind(&GazeboModelStatePublisher::read_state_callback, this)
  );

	// PUBLISHERS
	publisher_model_state = this->create_publisher<geometry_msgs::msg::Pose>(
    "/gazebo/model_state", qos_sensor_data
  );

	// CLIENTS
	client_model_state = this->create_client<gazebo_msgs::srv::GetModelState>(
    "/gazebo/get_model_state"
  );
}

GazeboModelStatePublisher::~GazeboModelStatePublisher() {}

void GazeboModelStatePublisher::read_state_callback() {
	RCLCPP_INFO(this->get_logger(), "Service calling");
	auto request = std::make_shared<gazebo_msgs::srv::GetModelState::Request>();
	request->model_name = model_name;
	auto result = client_model_state->async_send_request(
    request, std::bind(
      &GazeboModelStatePublisher::publish_state_callback, 
      this, std::placeholders::_1
      )
  );
}

void GazeboModelStatePublisher::publish_state_callback(rclcpp::Client<gazebo_msgs::srv::GetModelState>::SharedFuture future) {
	if (future.get()) {
		auto model_state = future.get()->pose;
		publisher_model_state->publish(model_state);
	}
	else {
		RCLCPP_ERROR(this->get_logger(), "Service call failed");
	}
}

// MAIN
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GazeboModelStatePublisher>("x500_vision_0"));
	rclcpp::shutdown();
}
