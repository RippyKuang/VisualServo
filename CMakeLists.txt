cmake_minimum_required(VERSION 3.10)
project(VisualServo VERSION 1.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -O3 -g3")
find_package(PkgConfig REQUIRED)


add_subdirectory(simulate)
add_executable(runVisualServo runVisualServo.cpp)
target_link_libraries(runVisualServo PUBLIC simulate)

target_include_directories(runVisualServo PUBLIC
                          "${PROJECT_BINARY_DIR}"
                          "${PROJECT_SOURCE_DIR}/simulate"
                          )
