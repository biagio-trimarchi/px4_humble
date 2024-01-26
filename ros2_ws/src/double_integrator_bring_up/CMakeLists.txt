cmake_minimum_required(VERSION 3.8)
project(double_integrator_bring_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

INSTALL(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
INSTALL(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
