project(six_wheel_robot_drivetrain)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Drivetrain plugin
add_library(six_wheel_robot_drivetrain SHARED drivetrain.cc)
target_link_libraries(six_wheel_robot_drivetrain ${GAZEBO_LIBRARIES})

# Example
add_executable(drive_example drive_example.cc)
target_link_libraries(drive_example ${GAZEBO_LIBRARIES})
