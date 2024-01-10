// LIBRARIES
// ROS2 Libraries
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ Standard Libraries
#include <chrono>
#include <cmath>

// Third Party Libraries
#include <Eigen/Dense>

// Custom Libraries
#include <double_integrator_test/logGPIS.hpp>

// CLASS
class LogGPISTest : public rclcpp::Node {
	public:
		LogGPISTest() : Node("Visualizer") {
			// Parameters
			// Timers
			timer_debug = this->create_wall_timer(
					std::chrono::milliseconds(1000),
					std::bind(&LogGPISTest::timerCallback, this)
				);
			// Subscribers
			
			// Publishers
			publisher_debug = this->create_publisher<visualization_msgs::msg::Marker>(
						"/log_gpis", 10
					);
			
			double lamba_whittle = 5.0;
			double resolution = 0.2;
			double error_variance = 0.01;
			log_gpis = LogGPIS(lamba_whittle, resolution, error_variance);

			addGroundSphereCylinder();
		}
		~LogGPISTest() {}
	
	private:
		// Functions
		void addGroundSphereCylinder() {
			// Add ground
			for (double x = -5.0; x < 5.1; x += 0.5) {
				for (double y = -5.0; y < 5.1; y += 0.5) {
					Eigen::Vector3d point(x, y, 0.0);
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
					point = cylinder_center_ground + point;
					log_gpis.add_sample(point);
				}
			}

			RCLCPP_INFO(this->get_logger(), "Training...");
			log_gpis.train();
			RCLCPP_INFO(this->get_logger(), "Done!");
		}

		void scanRegion() {
			double scan_resolution = 0.2;
			for (double x = -5.0; x < 5.1; x+=scan_resolution){
				for (double y = -5.0; y < 5.1; y+=scan_resolution){
					for (double z = -5.0; z < 6.1; z+=scan_resolution){
						Eigen::Vector3d scan_point(x, y, z);
						if (log_gpis.evaluate(scan_point) < 0.1) {
							geometry_msgs::msg::Point point;
							point.x = x;
							point.y = y;
							point.z = z;
							debug_message.points.push_back(point);
						}
					}
				}
			}
		}

		void timerCallback() {
			RCLCPP_INFO(this->get_logger(), "Preparing message...");

			debug_message.header.frame_id = "map";
			debug_message.header.stamp = this->get_clock()->now();
			debug_message.type = visualization_msgs::msg::Marker::POINTS;
			debug_message.action = visualization_msgs::msg::Marker::ADD;
			debug_message.points.clear();
			scanRegion();
			debug_message.pose.position.x = 0.0;
			debug_message.pose.position.y = 0.0;
			debug_message.pose.position.z = 0.0;
			debug_message.pose.orientation.w = 1.0;
			debug_message.pose.orientation.x = 0.0;
			debug_message.pose.orientation.y = 0.0;
			debug_message.pose.orientation.z = 0.0;
			debug_message.scale.x = 0.1;
			debug_message.scale.y = 0.1;
			debug_message.scale.z = 0.1;
			debug_message.color.r = 0.0;
			debug_message.color.g = 1.0;
			debug_message.color.b = 0.0;
			debug_message.color.a = 1.0;

			publisher_debug->publish(debug_message);
			RCLCPP_INFO(this->get_logger(), "Message sent!");
		}

		// Variables
		LogGPIS log_gpis;
		visualization_msgs::msg::Marker debug_message;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_debug;
		rclcpp::TimerBase::SharedPtr timer_debug;

};

// MAIN
int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LogGPISTest>());
	rclcpp::shutdown();
	return 0;
}
