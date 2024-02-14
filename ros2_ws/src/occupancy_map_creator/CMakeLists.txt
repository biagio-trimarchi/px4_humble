cmake_minimum_required(VERSION 3.8)
project(occupancy_map_creator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
		${PROJECT_SOURCE_DIR}/include
)

add_executable(ldc_map_creator src/occupancy_map_creator)
ament_target_dependencies(ldc_map_creator
	rclcpp nav_msgs
)

install(TARGETS ldc_map_creator DESTINATION lib/${PROJECT_NAME})

ament_package()
