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
find_package(gazebo_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIRS}
	${QPOASES_INCLUDE_DIRS}
)

add_executable(governor_node src/double_integrator.cpp src/bezier_utilities.cpp src/trajectory.cpp src/logGPIS.cpp src/gp_matern32)
ament_target_dependencies(governor_node
	rclcpp px4_msgs Eigen3 std_msgs visualization_msgs geometry_msgs tf2 tf2_ros
)
target_link_libraries(governor_node qpOASES)

add_executable(governor_node_2 src/double_integrator_2.cpp src/bezier_utilities.cpp src/trajectory.cpp src/logGPIS.cpp src/gp_matern32)
ament_target_dependencies(governor_node_2
	rclcpp px4_msgs Eigen3 std_msgs visualization_msgs geometry_msgs tf2 tf2_ros
)
target_link_libraries(governor_node_2 qpOASES)

add_executable(gazebo_pose_publisher src/model_state_publisher.cpp)
ament_target_dependencies(gazebo_pose_publisher
	rclcpp gazebo_msgs
)

add_executable(visualization_test src/sphere_and_cylinder_test.cpp src/logGPIS.cpp src/gp_matern32.cpp)
ament_target_dependencies(visualization_test 
	rclcpp Eigen3 visualization_msgs geometry_msgs
)

install( TARGETS governor_node DESTINATION lib/${PROJECT_NAME})
install( TARGETS governor_node_2 DESTINATION lib/${PROJECT_NAME})
install( TARGETS gazebo_pose_publisher DESTINATION lib/${PROJECT_NAME})
install( TARGETS visualization_test DESTINATION lib/${PROJECT_NAME})

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
