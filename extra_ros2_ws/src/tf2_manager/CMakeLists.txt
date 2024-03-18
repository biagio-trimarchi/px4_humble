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

# REALSENSE TEST
add_executable(realsense_test src/realsense_test.cpp)
target_include_directories(realsense_test PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(realsense_test
  rclcpp std_msgs geometry_msgs 
  px4_msgs Eigen3 tf2 
  tf2_ros nav_msgs
)
INSTALL( TARGETS realsense_test
  DESTINATION lib/${PROJECT_NAME}
)

# LDC MANAGER
add_executable(ldc_tf_manager src/ldc_tf_manager.cpp)
target_include_directories(realsense_test PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(ldc_tf_manager
  rclcpp std_msgs geometry_msgs 
  px4_msgs Eigen3 tf2 
  tf2_ros nav_msgs
)
INSTALL( TARGETS ldc_tf_manager
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
