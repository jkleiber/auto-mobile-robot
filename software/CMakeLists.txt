project(auto_mobile_robot)

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

include(CheckIncludeFile)
include(CheckIncludeFileCXX)
include(CheckIncludeFiles)

#${CMAKE_CURRENT_SOURCE_DIR}/include 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# add_subdirectory(include)
add_subdirectory(src)
