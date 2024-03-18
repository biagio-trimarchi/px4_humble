// PREPROCESSOR DIRECTIVES

// LIBRARIES

// ROS Libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// Cpp Standard Libraries


// Other Libraries
#include <Eigen/Eigen>

// 
using namespace std::chrono_literals;
using std::placeholders::_1;

// Classes
class LDC_tf_manager : public rclcpp::Node {
  public:
    LDC_tf_manager() : Node("ldc_tf_manager") {
      // PARAMS
      // Set parameters
			this->declare_parameter("tracking_camera_orientation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
			this->declare_parameter("tracking_camera_translation", std::vector<double>{0.0, 0.0, 0.0});
			this->declare_parameter("depth_camera_orientation", std::vector<double>{1.0, 0.0, 0.0, 0.0});
			this->declare_parameter("depth_camera_translation", std::vector<double>{0.0, 0.0, 0.0});
			this->declare_parameter("initial_position", std::vector<double>{0.0, 0.0, 0.0});
			this->declare_parameter("map_frame", "map");
			this->declare_parameter("drone_frame", "drone");
			this->declare_parameter("tracking_frame", "T265_link");
			this->declare_parameter("depth_frame", "D400_link");
      
      // Get parameters
			rclcpp::Parameter param_initial_position = this->get_parameter("initial_position");
			rclcpp::Parameter param_tracking_camera_orientation = this->get_parameter("tracking_camera_orientation");
			rclcpp::Parameter param_trackin_camera_position = this->get_parameter("tracking_camera_translation");
			rclcpp::Parameter param_depth_camera_orientation = this->get_parameter("depth_camera_orientation");
			rclcpp::Parameter param_depth_camera_position = this->get_parameter("depth_camera_translation");
			this->get_parameter("drone_frame", drone_frame);
			this->get_parameter("tracking_frame", tracking_frame);
			this->get_parameter("depth_frame", depth_frame);

      // QOS
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      // SUBSCRIBERS
      odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/drone_odometry", qos, std::bind(&LDC_tf_manager::odometry_callback, this, _1)
          );
      
      // PUBLISHERS

      // TF2
      tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    
  private:
    // FUNCTIONS
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

			// Read odometry
      map_to_drone_translation = Eigen::Vector3d(
			  msg->pose.pose.position.x,
			  msg->pose.pose.position.y,
			  msg->pose.pose.position.z);
      map_to_drone_orientation = Eigen::Quaterniond(
			  msg->pose.pose.orientation.w,
			  msg->pose.pose.orientation.x,
			  msg->pose.pose.orientation.y,
			  msg->pose.pose.orientation.z); 

      broadcastTF2();  
    }

    void broadcastTF2() {
      geometry_msgs::msg::TransformStamped t_msg; 
      rclcpp::Time transform_time_stamp;

      transform_time_stamp = this->get_clock()->now();
			
			// Map to drone
      t_msg.header.frame_id = map_frame;
      t_msg.child_frame_id = drone_frame;

      t_msg.transform.translation.x = map_to_drone_translation(0);
      t_msg.transform.translation.y = map_to_drone_translation(1);
      t_msg.transform.translation.z = map_to_drone_translation(2);

      t_msg.transform.rotation.w = map_to_drone_orientation.w();
      t_msg.transform.rotation.x = map_to_drone_orientation.x();
      t_msg.transform.rotation.y = map_to_drone_orientation.y();
      t_msg.transform.rotation.z = map_to_drone_orientation.z(); 

      tf2_broadcaster->sendTransform(t_msg);
      
      // Drone to tracking camera
      t_msg.header.frame_id = drone_frame;
      t_msg.child_frame_id = tracking_frame;

      t_msg.transform.translation.x = drone_to_tracking_camera_translation(0);
      t_msg.transform.translation.y = drone_to_tracking_camera_translation(1);
      t_msg.transform.translation.z = drone_to_tracking_camera_translation(2);

      t_msg.transform.rotation.w = drone_to_tracking_camera_orientation.w();
      t_msg.transform.rotation.x = drone_to_tracking_camera_orientation.x();
      t_msg.transform.rotation.y = drone_to_tracking_camera_orientation.y();
      t_msg.transform.rotation.z = drone_to_tracking_camera_orientation.z(); 

      tf2_broadcaster->sendTransform(t_msg);
      
      // Drone to depth camera
      t_msg.header.frame_id = drone_frame;
      t_msg.child_frame_id = depth_frame;

      t_msg.transform.translation.x = drone_to_depth_camera_translation(0);
      t_msg.transform.translation.y = drone_to_depth_camera_translation(1);
      t_msg.transform.translation.z = drone_to_depth_camera_translation(2);

      t_msg.transform.rotation.w = drone_to_depth_camera_orientation.w();
      t_msg.transform.rotation.x = drone_to_depth_camera_orientation.x();
      t_msg.transform.rotation.y = drone_to_depth_camera_orientation.y();
      t_msg.transform.rotation.z = drone_to_depth_camera_orientation.z(); 

      tf2_broadcaster->sendTransform(t_msg);
    }

    // VARIABLES  
    Eigen::Vector3d map_to_drone_translation = Eigen::Vector3d::Zero();                      // Drone position 
    Eigen::Quaterniond map_to_drone_orientation = Eigen::Quaterniond::Identity();            // Drone rotation 
    Eigen::Vector3d drone_to_tracking_camera_translation = Eigen::Vector3d::Zero();           // Tracking camera position wrt to drone
    Eigen::Quaterniond drone_to_tracking_camera_orientation = Eigen::Quaterniond::Identity(); // Tracking camera rotation wrt to drone					
    Eigen::Vector3d drone_to_depth_camera_translation = Eigen::Vector3d::Zero();              // Depth camera position wrt to drone
    Eigen::Quaterniond drone_to_depth_camera_orientation = Eigen::Quaterniond::Identity();    // Depth camera orientation wrt to drone

		std::string map_frame;
		std::string drone_frame;
		std::string tracking_frame;
		std::string depth_frame;

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;

    // Publishers

    // TF2 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster; 
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LDC_tf_manager>());
  rclcpp::shutdown();
  return 0;
}
