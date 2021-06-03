project(kleiber_control)

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPTIMIZATION REQUIRED nlopt)

add_library(${PROJECT_NAME} src/ramsete.cpp src/point_shoot.cpp src/pid_controller.cpp src/utils.cpp src/linear_mpc.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} 
    PUBLIC ${OPTIMIZATION_LIBRARIES} kleiber_guidance kleiber_navigation)
