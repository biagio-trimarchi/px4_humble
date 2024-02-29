#include <camera_to_occupancy/save_cloud.hpp>

CameraToOccupancy::CameraToOccupancy() : Node("CameraToOccupancy") {
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
	
	// Subscriptions
	subscriber_pointcloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
	  "/in_cloud", qos_sensor_data,
	  std::bind(&CameraToOccupancy::pointcloudCallback, this, std::placeholders::_1)
	  );
}

CameraToOccupancy::~CameraToOccupancy() {}

void CameraToOccupancy::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pointcloud) {
	pcl::PCDWriter writer;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*msg_pointcloud, *cloud);
	writer.write<pcl::PointXYZ>("test_cloud.pcd", *cloud, false);
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CameraToOccupancy>());
	rclcpp::shutdown();
	return 0;
}
