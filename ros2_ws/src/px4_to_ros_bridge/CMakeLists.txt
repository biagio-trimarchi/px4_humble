cmake_minimum_required(VERSION 3.5)
project(px4_to_ros_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(nav_msgs REQUIRED)

# Add executables
add_executable(px4_msgs_bridge src/px4_msgs_bridge.cpp)

# Link dependencies to target
ament_target_dependencies(px4_msgs_bridge px4_ros_com rclcpp px4_msgs nav_msgs)

# Install target in the installation directory
install(
	TARGETS
	px4_msgs_bridge

	DESTINATION
	lib/${PROJECT_NAME}
)	

ament_package()
