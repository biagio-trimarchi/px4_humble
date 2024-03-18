cmake_minimum_required(VERSION 3.5)
project(visual_to_px4)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)

# Add executable
add_executable(T265_to_px4 src/T265_to_px4.cpp)
target_include_directories(T265_to_px4 PUBLIC
  ${EIGEN3_INCLUDE_DIR}
)
# Dependencies
ament_target_dependencies(T265_to_px4 rclcpp px4_msgs nav_msgs Eigen3)
# Install
install(TARGETS
	T265_to_px4
	DESTINATION lib/${PROJECT_NAME})

ament_package()
