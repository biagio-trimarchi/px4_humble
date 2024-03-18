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
class RealsenseManager : public rclcpp::Node {
  public:
    RealsenseManager() : Node("realsense_test") {
      // PARAMS
      // Set parameters
      
      // Get parameters

      // QOS
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      // SUBSCRIBERS
      realsense_odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "/camera/pose/sample", qos, std::bind(&RealsenseManager::camera_odometry_callback, this, _1)
          );
      
      // PUBLISHERS

      // TF2
      tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    
  private:
    // FUNCTIONS
    void camera_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

      odom_to_drone_translation = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
      camera_to_odom_rotation = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z); 
      drone_to_odom_rotation = drone_to_camera_rotation * camera_to_odom_rotation;
      broadcastTF2();  
    }

    void broadcastTF2() {
      geometry_msgs::msg::TransformStamped t_odom_to_drone; 
      rclcpp::Time transform_time_stamp;

      transform_time_stamp = this->get_clock()->now();
      // map_to_odom
      t_odom_to_drone.header.frame_id = "odom_frame";
      t_odom_to_drone.child_frame_id = "drone";

      t_odom_to_drone.transform.translation.x = odom_to_drone_translation(0);
      t_odom_to_drone.transform.translation.y = odom_to_drone_translation(1);
      t_odom_to_drone.transform.translation.z = odom_to_drone_translation(2);

      t_odom_to_drone.transform.rotation.w = drone_to_odom_rotation.w();
      t_odom_to_drone.transform.rotation.x = drone_to_odom_rotation.x();
      t_odom_to_drone.transform.rotation.y = drone_to_odom_rotation.y();
      t_odom_to_drone.transform.rotation.z = drone_to_odom_rotation.z(); 

      tf2_broadcaster->sendTransform(t_odom_to_drone);
    }

    // VARIABLES  
    Eigen::Vector3d odom_to_drone_translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond drone_to_odom_rotation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond camera_to_odom_rotation = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond drone_to_camera_rotation = Eigen::Quaterniond::Identity();
    // Eigen::Quaterniond drone_to_camera_rotation = Eigen::Quaterniond(0.9238795, 0.0, -0.3826834, 0.0);

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr realsense_odometry_subscriber;

    // Publishers

    // TF2 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster; 
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealsenseManager>());
  rclcpp::shutdown();
  return 0;
}
