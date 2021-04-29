project(kleiber_guidance)

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Once code exists, add it and set INTERFACE to public in target_include_directories
add_library(${PROJECT_NAME} src/trajectory_reader.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC kleiber_navigation)
