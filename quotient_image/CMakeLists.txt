cmake_minimum_required(VERSION 3.10)
project(quotient_image)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_IGNORE_PATH /local1)

find_package(OpenCV REQUIRED)

add_executable(quotient_image main.cpp)
target_link_libraries(quotient_image ${OpenCV_LIBS})
