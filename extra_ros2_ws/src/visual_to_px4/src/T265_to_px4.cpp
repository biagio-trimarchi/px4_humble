// C++ Standard Libraries
#include <functional>
#include <memory>
#include <string>
#include <cmath>

// ROS Stuff
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

// Other Libraries
#include <Eigen/Eigen>

// USING
using std::placeholders::_1;

class T265ToPX4 : public rclcpp::Node{
	public:
		T265ToPX4() : Node("T265_to_px4_bridge") {
			// Declare parameters
			this->declare_parameter("debug", false);
			this->declare_parameter("initial_position", std::vector<double>{0.0, 0.0, 0.0});
			this->declare_parameter("drone_to_camera_orientation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
			this->declare_parameter("camera_translation", std::vector<double>{0.0, 0.0, 0.0});

			// Get Parameters
			rclcpp::Parameter param_initial_position = this->get_parameter("initial_position");
			rclcpp::Parameter param_drone_to_camera_orientation = this->get_parameter("drone_to_camera_orientation");
			rclcpp::Parameter param_camera_position = this->get_parameter("camera_translation");
			this->get_parameter("debug", debug);

			// Initialize variables
			is_orientation_estimated = false;
			moving_average_vector_dimension = 10;
			for (unsigned int i = 0; i < moving_average_vector_dimension; ++i) {
				moving_average_vector_rotation_around_x.push_back(0.0);
				moving_average_vector_rotation_around_y.push_back(0.0);
			}
			estimated_camera_orientation = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

			initial_position = Eigen::Vector3d(
				(param_initial_position.as_double_array())[0], // x
				(param_initial_position.as_double_array())[1], // y
				(param_initial_position.as_double_array())[2]  // z
			);

			drone_to_camera_orientation = Eigen::Quaterniond(
				(param_drone_to_camera_orientation.as_double_array())[0], // w
				(param_drone_to_camera_orientation.as_double_array())[1], // x
				(param_drone_to_camera_orientation.as_double_array())[2], // y
				(param_drone_to_camera_orientation.as_double_array())[3]  // z
			);

			camera_position = Eigen::Vector3d(
				(param_camera_position.as_double_array())[0], // x
				(param_camera_position.as_double_array())[1], // y
				(param_camera_position.as_double_array())[2]  // z
			);

			// QoS
			rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
			auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
      // Subscribtions
			subscription_odometry = this->create_subscription<nav_msgs::msg::Odometry>("/camera/pose/sample", qos, std::bind(&T265ToPX4::subscription_callback, this, _1));
			subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>("/camera/imu", qos, std::bind(&T265ToPX4::estimate_orientation, this, _1));

      // Publishers 
			publisher_odometry_px4 = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", qos);
		}

	private:
		void subscription_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {
			if (!is_orientation_estimated) {
				return;
			}
			// Visual Odometry message to be sent to the px4 to update the EKF2
			auto msg_pub = px4_msgs::msg::VehicleOdometry();
			
			// Set reference frame
			// FRD is a reference frame world fixed pointed in the initial
			// configuration of the vehicle 
			// TODO: Test FRD e NED
			msg_pub.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD; 
			// msg_pub.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED; 

      // Rotation from camera orientation to drone orientation
      Eigen::Vector3d odom_to_camera_translation(
			  msg->pose.pose.position.x,
			  msg->pose.pose.position.y, 
			  msg->pose.pose.position.z);
      Eigen::Quaterniond odom_to_drone_to_camera_orientation(
			  msg->pose.pose.orientation.w,
			  msg->pose.pose.orientation.x, 
			  msg->pose.pose.orientation.y,
			  msg->pose.pose.orientation.z);
      Eigen::Vector3d odom_to_drone_translation;
      Eigen::Quaterniond odom_to_drone_orientation;

      odom_to_drone_orientation = odom_to_drone_to_camera_orientation * drone_to_camera_orientation.conjugate() * estimated_camera_orientation;
      odom_to_drone_translation = odom_to_camera_translation - odom_to_drone_orientation * camera_position;
			
			// For more info about the conversion see
			// https://docs.px4.io/main/en/ros/external_position_estimation.html
			// Conversion from FLU to FRD
			
			// Position
			msg_pub.position[0] =   odom_to_drone_translation.x() + initial_position.x();
			msg_pub.position[1] = - (odom_to_drone_translation.y() + initial_position.y());
			msg_pub.position[2] = - (odom_to_drone_translation.z() + initial_position.z());
			RCLCPP_INFO(this->get_logger(), "Posiiton x: %f, y: %f, z: %f", 
			            msg_pub.position[0], -msg_pub.position[1], -msg_pub.position[2] );
			
			// Orientation 
			msg_pub.q[0] =   odom_to_drone_orientation.w();	
			msg_pub.q[1] =   odom_to_drone_orientation.x();
			msg_pub.q[2] = - odom_to_drone_orientation.y();
			msg_pub.q[3] = - odom_to_drone_orientation.z();
			
			// The other components does not need to be set
			// since we can only give to the PX4 EKF the position and 
			// the orientation of the vehicle.
			// The right parameters need to be set on the firmware
			// in order to fuse onòy this information
			
			publisher_odometry_px4->publish(msg_pub);

			if (debug) {
				RCLCPP_INFO(this->get_logger(), "Message published!");
			}
		}

		void estimate_orientation(const sensor_msgs::msg::Imu::SharedPtr msg) {
			Eigen::Vector3d readed_gravity(-msg->linear_acceleration.z, -msg->linear_acceleration.x, -msg->linear_acceleration.y);
			readed_gravity /= readed_gravity.norm();

			double moving_average_vector_rotation_around_x_sum = 0.0;
			double moving_average_vector_rotation_around_y_sum = 0.0;
			for (unsigned int i = 0; i < moving_average_vector_dimension - 1; ++i) {
				moving_average_vector_rotation_around_x[i] = moving_average_vector_rotation_around_x[i+1];
				moving_average_vector_rotation_around_y[i] = moving_average_vector_rotation_around_y[i+1];

				moving_average_vector_rotation_around_x_sum += moving_average_vector_rotation_around_x[i];
				moving_average_vector_rotation_around_y_sum += moving_average_vector_rotation_around_y[i];
			}

			moving_average_vector_rotation_around_y[moving_average_vector_dimension] = std::atan2(-readed_gravity.x(), readed_gravity.z());
			double temporary_quantity_a = -readed_gravity.x() * std::sin(moving_average_vector_rotation_around_y[moving_average_vector_dimension]) + readed_gravity.z() * std::cos(moving_average_vector_rotation_around_y[moving_average_vector_dimension]);
			moving_average_vector_rotation_around_x[moving_average_vector_dimension] = std::atan2(readed_gravity.y(), temporary_quantity_a);


			
			double rotation_around_y = moving_average_vector_rotation_around_y_sum / moving_average_vector_dimension;
			double rotation_around_x = moving_average_vector_rotation_around_x_sum / moving_average_vector_dimension;

			Eigen::Quaterniond q_rotation_around_y(std::cos(rotation_around_y / 2.0), // w
			                                       0.0,                               // x
			                                       std::sin(rotation_around_y / 2.0), // y
			                                       0.0);                              // z
																																							 //
			Eigen::Quaterniond q_rotation_around_x(std::cos(rotation_around_x / 2.0), // w
			                                       std::sin(rotation_around_x / 2.0), // x 
			                                       0.0,                               // y
			                                       0.0);                              // z
																																							 
			estimated_camera_orientation = q_rotation_around_x * q_rotation_around_y; // From camera frame to world frame!
																																								
			// Debug
			if (debug) {
				Eigen::Vector3d real_gravity = estimated_camera_orientation * readed_gravity;
				RCLCPP_INFO(this->get_logger(), "Readed x: %f, y: %f, z: %f", readed_gravity.x(), readed_gravity.y(), readed_gravity.z());
				RCLCPP_INFO(this->get_logger(), "Real gravity x: %f, y: %f, z: %f", real_gravity.x(), real_gravity.y(), real_gravity.z());
			}

			is_orientation_estimated = true;
		}
	
		// Variables
		bool debug;
		Eigen::Quaterniond drone_to_camera_orientation;
		Eigen::Vector3d camera_position;
		Eigen::Vector3d initial_position;
		bool is_orientation_estimated;
		unsigned int moving_average_vector_dimension;
		std::vector<double> moving_average_vector_rotation_around_x;
		std::vector<double> moving_average_vector_rotation_around_y;
		Eigen::Quaterniond estimated_camera_orientation;
		
		// Subscriptions
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_odometry;
		rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;

		// Publishers
		rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_odometry_px4;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<T265ToPX4>());
	rclcpp::shutdown();

	return 0;
}
