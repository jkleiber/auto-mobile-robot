project(kleiber_navigation)

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

add_library(${PROJECT_NAME} src/diffdrive_ekf.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC kleiber_control)