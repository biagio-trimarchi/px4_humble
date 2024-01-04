cmake_minimum_required(VERSION 3.8)
project(log_gpis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
	scripts/logGPIS_node.py
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
