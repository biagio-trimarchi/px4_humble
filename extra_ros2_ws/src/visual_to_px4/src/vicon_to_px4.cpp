// C++ Standard Libraries
#include <functional>
#include <memory>
#include <string>

// ROS Stuff
#include <rclcpp/rclcpp.hpp>
#include <vicon_receiver/msg/position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

// USING
using std::placeholders::_1;

class ViconToPX4 : public rclcpp::Node{
	public:
		ViconToPX4() : Node("vicon_to_px4_bridge") {

			publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("fmu/in/vehicle_visual_odometry", 10);
			subscription_ = this->create_subscription<vicon_receiver::msg::Position>("/vicon/LeoDrone/LeoDrone", 10, std::bind(&ViconToPX4::subscription_callback, this, _1));
		}

	private:
		void subscription_callback(const vicon_receiver::msg::Position::SharedPtr msg) const {
			// Visual Odometry message to be sent to the px4 to update the EKF2
			auto msg_pub = px4_msgs::msg::VehicleOdometry();
			
			// Set reference frame
			// FRD is a reference frame world fixed pointed in the initial
			// configuration of the vehicle (which should be the same as the vicon 
			// reference frame)
			msg_pub.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD; 
			
			// For more info about the conversion see
			// https://docs.px4.io/main/en/ros/external_position_estimation.html
			
			// Position
			msg_pub.position[0] =   msg->x_trans * 0.001;
			msg_pub.position[1] = - msg->y_trans * 0.001;
			msg_pub.position[2] = - msg->z_trans * 0.001;
			
			// Orientation 
			msg_pub.q[0] =   msg->w;	
			msg_pub.q[1] =   msg->x_rot;	
			msg_pub.q[2] = - msg->y_rot;	
			msg_pub.q[3] = - msg->z_rot;	
			
			// The other components does not need to be set
			// since we can only give to the PX4 EKF the position and 
			// the orientation of the vehicle.
			// The right parameters need to be set on the firmware
			// in order to fuse onòy this information
			
			publisher_->publish(msg_pub);

			return;		
		}
		
		rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
		rclcpp::Subscription<vicon_receiver::msg::Position>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ViconToPX4>());
	rclcpp::shutdown();

	return 0;
}
