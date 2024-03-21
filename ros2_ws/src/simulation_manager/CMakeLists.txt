cmake_minimum_required(VERSION 3.8)
project(simulation_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(log_gpis REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIRS}
	${QPOASES_INCLUDE_DIRS}
)

add_library(world_utilities src/world_manager.cpp)
target_link_libraries(world_utilities log_gpis::logGPIS)
add_executable(gazebo_manager src/gazebo_manager.cpp)
target_link_libraries(gazebo_manager log_gpis::logGPIS world_utilities)
ament_target_dependencies(gazebo_manager 
	rclcpp geometry_msgs visualization_msgs 
	nav_msgs tf2 tf2_ros 
	px4_msgs px4_ros_com
	Eigen3 log_gpis)

install(TARGETS gazebo_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

# Test
add_executable(test_node src/double_integrator_test.cpp)
target_link_libraries(test_node log_gpis::logGPIS world_utilities)
ament_target_dependencies(test_node
	rclcpp geometry_msgs visualization_msgs 
	std_msgs Eigen3 log_gpis)
target_link_libraries(test_node qpOASES)
install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
