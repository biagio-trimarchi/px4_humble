cmake_minimum_required(VERSION 3.5)
project(tf2_manager)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(tf2_gazebo_manager_node src/tf2_gazebo_manager.cpp)

target_include_directories(tf2_gazebo_manager_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(tf2_gazebo_manager_node
  rclcpp
  std_msgs 
  geometry_msgs 
  px4_msgs 
  Eigen3 
  tf2 
  tf2_ros
  px4_ros_com
  nav_msgs
)

INSTALL( TARGETS tf2_gazebo_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
