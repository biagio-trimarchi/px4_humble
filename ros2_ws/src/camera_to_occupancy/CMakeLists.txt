cmake_minimum_required(VERSION 3.8)
project(camera_to_occupancy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED) 

include_directories(
  ${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions($(PCL_DEFINITIONS))

add_executable(camera_to_occupancy src/camera_to_occupancy.cpp)

ament_target_dependencies(camera_to_occupancy
	rclcpp nav_msgs sensor_msgs Eigen3 PCL pcl_conversions 
)

target_link_libraries(camera_to_occupancy
	${PCL_LIBRARIES}
)

INSTALL(TARGETS camera_to_occupancy
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
