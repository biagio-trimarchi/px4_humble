cmake_minimum_required(VERSION 3.8)
project(log_gpis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${EIGEN3_INCLUDE_DIRS}
)

# Create Interface
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/QueryEstimate.srv"
)

# Add Library
add_library(logGPIS SHARED src/logGPIS.cpp src/gp_matern32.cpp)
ament_target_dependencies(logGPIS Eigen3)
ament_export_targets(logGPIS HAS_LIBRARY_TARGET)

# Install
install(
	DIRECTORY include/log_gpis
	DESTINATION include
)

install(
	TARGETS logGPIS
	EXPORT logGPIS
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION	bin 
	INCLUDES DESTINATION include
)

ament_package()
