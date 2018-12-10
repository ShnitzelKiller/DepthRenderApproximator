cmake_minimum_required(VERSION 2.8)
project(depth_grid_render)

add_subdirectory(src/test)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_IGNORE_PATH /local1)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(depth_grid_render src/main.cpp src/CameraUtils.hpp src/XMLWriter.hpp src/OBJWriter.hpp src/OptionParser.hpp src/MeshBuilder.hpp src/typedefs.hpp)
target_link_libraries(depth_grid_render ${OpenCV_LIBS})
