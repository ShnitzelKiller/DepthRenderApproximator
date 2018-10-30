cmake_minimum_required(VERSION 3.10)
project(depth_grid_render)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_IGNORE_PATH /local1)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(depth_grid_render main.cpp CameraUtils.hpp)
message(STATUS ${Boost_LIBRARIES})
target_link_libraries(depth_grid_render ${OpenCV_LIBS} ${Boost_LIBRARIES})
