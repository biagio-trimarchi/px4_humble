cmake_minimum_required(VERSION 3.8)
project(double_integrator_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(log_gpis REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIRS}
	${QPOASES_INCLUDE_DIRS}
)

add_executable(governor_node_2 src/double_integrator_2.cpp src/bezier_utilities.cpp src/trajectory.cpp)
ament_target_dependencies(governor_node_2
	rclcpp px4_msgs Eigen3 
	std_msgs visualization_msgs geometry_msgs 
	nav_msgs tf2 tf2_ros 
	log_gpis
)
target_link_libraries(governor_node_2 qpOASES)

add_executable(governor_node_acceleration src/double_integrator_acceleration.cpp src/bezier_utilities.cpp src/trajectory.cpp)
ament_target_dependencies(governor_node_acceleration
	rclcpp px4_msgs Eigen3 
	std_msgs visualization_msgs geometry_msgs 
	nav_msgs tf2 tf2_ros 
	log_gpis
)
target_link_libraries(governor_node_acceleration qpOASES)

add_executable(governor_node_moving_obs src/double_integrator_moving_obs.cpp src/bezier_utilities.cpp src/trajectory.cpp)
ament_target_dependencies(governor_node_moving_obs
	rclcpp px4_msgs Eigen3 
	std_msgs visualization_msgs geometry_msgs 
	nav_msgs tf2 tf2_ros 
	log_gpis
)
target_link_libraries(governor_node_moving_obs qpOASES)

#add_executable(governor_takeoff src/test_takeoff.cpp src/bezier_utilities.cpp src/trajectory.cpp)
add_executable(governor_takeoff src/test_takeoff.cpp src/bezier_utilities.cpp src/trajectory.cpp)
ament_target_dependencies(governor_takeoff
	rclcpp px4_msgs Eigen3 
	std_msgs visualization_msgs geometry_msgs 
	nav_msgs tf2 tf2_ros 
)

add_executable(governor_node_multi src/test_multi_gpis.cpp)
ament_target_dependencies(governor_node_multi
	rclcpp px4_msgs Eigen3 
	std_msgs visualization_msgs geometry_msgs 
	nav_msgs tf2 tf2_ros 
	log_gpis
)
target_link_libraries(governor_node_multi qpOASES)

install( TARGETS governor_node_2 DESTINATION lib/${PROJECT_NAME})
install( TARGETS governor_node_acceleration DESTINATION lib/${PROJECT_NAME})
install( TARGETS governor_takeoff DESTINATION lib/${PROJECT_NAME})
install( TARGETS governor_node_moving_obs DESTINATION lib/${PROJECT_NAME})
install( TARGETS governor_node_multi DESTINATION lib/${PROJECT_NAME})

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
