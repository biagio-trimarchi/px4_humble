#include <occupancy_map_creator/occupancy_map_creator.hpp>

LDCMapCreator::LDCMapCreator() : Node("mapNode") {
	// Set parameters
	this->declare_parameter("map_resolution", 0.2);
	this->declare_parameter("map_width", 20.0);
	this->declare_parameter("map_height", 10.0);

	createMap();
	
	// Timers
	timer_map = this->create_wall_timer(
		std::chrono::milliseconds(200),
		std::bind(&LDCMapCreator::timerCallback, this)
	);
	
	// QoS
	rmw_qos_profile_t qos_profile_sensor_data = rmw_qos_profile_sensor_data;
	rclcpp::QoS qos_sensor_data = rclcpp::QoS(
	  rclcpp::QoSInitialization(qos_profile_sensor_data.history, 1),
	  qos_profile_sensor_data
	);

	// Publishers
	publisher_map = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
	  "/map", qos_sensor_data
	);
}

LDCMapCreator::~LDCMapCreator() {}

void LDCMapCreator::timerCallback() {
	RCLCPP_INFO(this->get_logger(), "Map published!");
	map.header.stamp = this->get_clock()->now();
	map.header.frame_id = "map";
	publisher_map->publish(map);
}

void LDCMapCreator::createMap() {
	this->get_parameter("map_resolution", map.info.resolution);
	map.info.width = int(std::floor(
		this->get_parameter("map_width").as_double() / this->get_parameter("map_resolution").as_double()
		)
	);
	map.info.height = int(std::floor(
		this->get_parameter("map_height").as_double() / this->get_parameter("map_resolution").as_double()
		)
	);

	map.info.origin.position.x = 0.0;
	map.info.origin.position.y = 0.0;
	map.info.origin.position.z = 0.0;

	map.info.origin.orientation.w = 1.0;
	map.info.origin.orientation.y = 0.0;
	map.info.origin.orientation.x = 0.0;
	map.info.origin.orientation.z = 0.0;

	map.data = std::vector<signed char>(map.info.width * map.info.height, -1);

	// Fill map
	for (unsigned int index = 0; index < map.info.width; index++) {
		map.data[index] = 1;
		map.data[map.info.width * map.info.height - 1 - index] = 1;
	}

	for (unsigned int index = 0; index < map.info.height; index++) {
		map.data[(map.info.width)*(index+1) - 1] = 1;
		map.data[(map.info.width)*index] = 1;
	}

	mapERF2024();

	RCLCPP_INFO(this->get_logger(), "Map created!");
}

void LDCMapCreator::mapERF2024() {
	// Builing to add                      Building number
	addRectangle(12.4, 0.00, 2.6, 1.5); // 09
	addRectangle(16.9, 1.00, 2.1, 2.1); // 06
	addRectangle(14.4, 7.65, 4.6, 1.7); // 10 (4?)
	addRectangle(12.8, 3.40, 0.6, 1.2); // 03 (Part1)
	addRectangle(13.4, 3.40, 1.0, 0.6); // 03 (Part2)
	addRectangle(14.4, 3.40, 0.6, 1.2); // 03 (Part3)
																			
	addRectangle(12.8, 5.40, 0.6, 1.2); // 03 (Part1)
	addRectangle(13.4, 6.00, 1.0, 0.6); // 03 (Part2)
	addRectangle(14.4, 5.40, 0.6, 1.2); // 03 (Part3)
	
}

void LDCMapCreator::addRectangle(double lower_left_corner_x,
                                 double lower_left_corner_y, 
                                 double lenght_x, double lenght_y) {
	for (double x = lower_left_corner_x; x < lower_left_corner_x + lenght_x; x+= map.info.resolution) {
		for (double y = lower_left_corner_y; y < lower_left_corner_y + lenght_y; y+= map.info.resolution) {
			int x_cell_coord = int(std::floor(x / map.info.resolution));
			int y_cell_coord = int(std::floor(y / map.info.resolution));

			if (x_cell_coord < 0 || (unsigned int) x_cell_coord >= map.info.width) {
				// Point outside map boundaries
				continue;
			}
			if (y_cell_coord < 0 || (unsigned int) y_cell_coord >= map.info.height) {
				// Point outside map boundaries
				continue;
			}
			map.data[x_cell_coord + y_cell_coord*map.info.width] = 1;
		}
	}
}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LDCMapCreator>());
	rclcpp::shutdown();
	return 0;
}
