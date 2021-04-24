project(amr_src)
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

add_subdirectory(control)
add_subdirectory(guidance)
add_subdirectory(navigation)


# C++17
set(CMAKE_CXX_STANDARD 17)

# LibSerial
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)


# Robot Code
add_executable(auto_mobile_robot main.cpp arduino_interface.cpp)
target_link_libraries(auto_mobile_robot PUBLIC kleiber_guidance kleiber_navigation kleiber_control PRIVATE ${SERIAL_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})



###################
#   Simulation
###################

# Gazebo Packages
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Simulator Code
add_executable(auto_mobile_robot_sim sim_main.cpp)
target_link_libraries(auto_mobile_robot_sim ${GAZEBO_LIBRARIES})
