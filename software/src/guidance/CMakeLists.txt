project(kleiber_guidance)

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# Once code exists, add it and set INTERFACE to public in target_include_directories
add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include)
