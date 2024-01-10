cmake_minimum_required(VERSION 3.0)

project(simple_qps)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS}
                    ${QPOASES_INCLUDE_DIRS})

add_executable(simple_qps src/main.cpp)
target_link_libraries(simple_qps qpOASES)
target_include_directories(simple_qps PUBLIC include)
