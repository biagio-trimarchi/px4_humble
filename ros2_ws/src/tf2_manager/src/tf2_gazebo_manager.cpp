// PREPROCESSOR DIRECTIVES

// LIBRARIES

// ROS Libraries
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_ros_com/frame_transforms.h>

// Cpp Standard Libraries


// Other Libraries
#include <Eigen/Eigen>

// 
using namespace std::chrono_literals;
using std::placeholders::_1;

// Classes
class GazeboPX4TFManager : public rclcpp::Node {
  public:
    GazeboPX4TFManager() : Node("tf2_gazebo_px4_manager") {
      // PARAMS
      // Set parameters
      this->declare_parameter("front_camera_position_x", 0.1);
      this->declare_parameter("front_camera_position_y", 0.0);
      this->declare_parameter("front_camera_position_z", 0.0);
      this->declare_parameter("front_camera_quaternion_w", 0.5);
      this->declare_parameter("front_camera_quaternion_x", -0.5);
      this->declare_parameter("front_camera_quaternion_y", 0.5);
      this->declare_parameter("front_camera_quaternion_z", -0.5);
      
      // Get parameters
      drone_to_front_camera_translation(0)  = this->get_parameter("front_camera_position_x").as_double(); 
      drone_to_front_camera_translation(1)  = this->get_parameter("front_camera_position_y").as_double(); 
      drone_to_front_camera_translation(2)  = this->get_parameter("front_camera_position_z").as_double(); 
      drone_to_front_camera_rotation.w() = this->get_parameter("front_camera_quaternion_w").as_double(); 
      drone_to_front_camera_rotation.x() = this->get_parameter("front_camera_quaternion_x").as_double(); 
      drone_to_front_camera_rotation.y() = this->get_parameter("front_camera_quaternion_y").as_double(); 
      drone_to_front_camera_rotation.z() = this->get_parameter("front_camera_quaternion_z").as_double(); 

      // QOS
      rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

      // SUBSCRIBERS
      uav_odometry_subscriber = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&GazeboPX4TFManager::vehicle_odometry_callback, this, _1)
          );
      
      // PUBLISHERS

      // TF2
      tf2_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      tf2_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    
  private:
    // FUNCTIONS
    void vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
      if ( std::isnan(msg->position[0]) || std::isnan(msg->position[1]) || std::isnan(msg->position[2]) 
          ||
          std::isnan(msg->velocity[0]) || std::isnan(msg->velocity[1]) || std::isnan(msg->velocity[2]) 
          ||
          std::isnan(msg->q[0])
        ) return;
      

      odom_to_drone_translation = px4_ros_com::frame_transforms::ned_to_enu_local_frame(Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]));
      odom_to_drone_rotation = px4_ros_com::frame_transforms::px4_to_ros_orientation(Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3])); 
           
      broadcastTF2();  
    }

    void broadcastTF2() {
      geometry_msgs::msg::TransformStamped t_map_to_odom; 
      geometry_msgs::msg::TransformStamped t_odom_to_drone; 
      geometry_msgs::msg::TransformStamped t_drone_to_front_camera; 
      rclcpp::Time transform_time_stamp;

      transform_time_stamp = this->get_clock()->now();
      // map_to_odom
      t_map_to_odom.header.stamp = transform_time_stamp;
      t_map_to_odom.header.frame_id = "map";
      t_map_to_odom.child_frame_id = "odom";

      t_map_to_odom.transform.translation.x = 0.0; 
      t_map_to_odom.transform.translation.y = 0.0;
      t_map_to_odom.transform.translation.z = 0.0; 

      t_map_to_odom.transform.rotation.w = 1.0;
      t_map_to_odom.transform.rotation.x = 0.0;
      t_map_to_odom.transform.rotation.y = 0.0;
      t_map_to_odom.transform.rotation.z = 0.0;

      // odom_to_drone
      t_odom_to_drone.header.stamp = transform_time_stamp;
      t_odom_to_drone.header.frame_id = "odom";
      t_odom_to_drone.child_frame_id = "drone";

      t_odom_to_drone.transform.translation.x = odom_to_drone_translation(0);
      t_odom_to_drone.transform.translation.y = odom_to_drone_translation(1);
      t_odom_to_drone.transform.translation.z = odom_to_drone_translation(2);

      t_odom_to_drone.transform.rotation.w = odom_to_drone_rotation.w();
      t_odom_to_drone.transform.rotation.x = odom_to_drone_rotation.x();
      t_odom_to_drone.transform.rotation.y = odom_to_drone_rotation.y();
      t_odom_to_drone.transform.rotation.z = odom_to_drone_rotation.z(); 

      // drone_to_camera 
      t_drone_to_front_camera.header.stamp = transform_time_stamp;
      t_drone_to_front_camera.header.frame_id = "drone";
      t_drone_to_front_camera.child_frame_id = "camera_link";

      t_drone_to_front_camera.transform.translation.x = drone_to_front_camera_translation(0);
      t_drone_to_front_camera.transform.translation.y = drone_to_front_camera_translation(1);
      t_drone_to_front_camera.transform.translation.z = drone_to_front_camera_translation(2);

      t_drone_to_front_camera.transform.rotation.w = drone_to_front_camera_rotation.w();
      t_drone_to_front_camera.transform.rotation.x = drone_to_front_camera_rotation.x();
      t_drone_to_front_camera.transform.rotation.y = drone_to_front_camera_rotation.y();
      t_drone_to_front_camera.transform.rotation.z = drone_to_front_camera_rotation.z(); 

      tf2_static_broadcaster->sendTransform(t_map_to_odom);
      tf2_static_broadcaster->sendTransform(t_drone_to_front_camera);
      tf2_broadcaster->sendTransform(t_odom_to_drone);
    }

    // VARIABLES  
    Eigen::Vector3d odom_to_drone_translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond odom_to_drone_rotation = Eigen::Quaterniond::Identity();
    Eigen::Vector3d drone_to_front_camera_translation = Eigen::Vector3d::Zero();
    Eigen::Quaterniond drone_to_front_camera_rotation = Eigen::Quaterniond::Identity();

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr uav_odometry_subscriber;

    // Publishers

    // TF2 
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf2_static_broadcaster; 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster; 
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboPX4TFManager>());
  rclcpp::shutdown();
  return 0;
}
