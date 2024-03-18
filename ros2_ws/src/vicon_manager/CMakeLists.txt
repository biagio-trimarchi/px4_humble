cmake_minimum_required(VERSION 3.8)
project(vicon_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vicon_receiver REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/include
)

add_executable(vicon_to_odometry src/vicon_to_odometry.cpp)
ament_target_dependencies(vicon_to_odometry
	rclcpp vicon_receiver
	nav_msgs geometry_msgs
	tf2_ros
)
install(TARGETS vicon_to_odometry
        DESTINATION lib/${PROJECT_NAME}
)

add_executable(T265_to_odometry src/T265_to_odometry.cpp)
ament_target_dependencies(T265_to_odometry
	rclcpp 
	nav_msgs geometry_msgs
	tf2_ros
)
install(TARGETS T265_to_odometry
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
