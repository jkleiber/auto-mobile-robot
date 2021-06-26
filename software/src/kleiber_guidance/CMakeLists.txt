project(kleiber_guidance)

cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Library compilation settings
add_library(${PROJECT_NAME} src/trajectory_reader.cpp src/pure_pursuit.cpp src/action.cpp src/vector_action.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC kleiber_navigation stdc++fs)
#target_compile_options(${PROJECT_NAME} PRIVATE "--std=c++17")
