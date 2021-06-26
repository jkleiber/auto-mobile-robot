project(amr_src)
cmake_minimum_required(VERSION 3.13 FATAL_ERROR)

add_subdirectory(kleiber_control)
add_subdirectory(kleiber_guidance)
add_subdirectory(kleiber_navigation)

######
# Compilation Options

OPTION(COMPILE_SIMULATION "Build the simulated robot code" ON)


# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# LibSerial
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)


# Robot Code
add_executable(auto_mobile_robot main.cpp arduino_interface.cpp loop.cpp)
#target_compile_options(auto_mobile_robot PRIVATE "--std=c++17")
target_link_libraries(auto_mobile_robot 
    PUBLIC 
        kleiber_guidance 
        kleiber_navigation 
        kleiber_control 
    PRIVATE 
        ${SERIAL_LIBRARIES} 
        ${CMAKE_THREAD_LIBS_INIT})



###################
#   Simulation
###################
if(COMPILE_SIMULATION)
    # Gazebo Packages
    find_package(gazebo REQUIRED)
    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

    # C++17
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Simulator Code
    add_executable(auto_mobile_robot_sim sim_main.cpp sim_interface.cpp loop.cpp)
    target_link_libraries(auto_mobile_robot_sim
        PUBLIC 
            kleiber_guidance 
            kleiber_navigation 
            kleiber_control 
            ${GAZEBO_LIBRARIES}
        PRIVATE 
            ${SERIAL_LIBRARIES} 
            ${CMAKE_THREAD_LIBS_INIT})
endif()


###########################
#         Testing
###########################

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPTIMIZATION REQUIRED nlopt)

add_executable(test_optimization optim_main.cpp)
target_link_libraries(test_optimization PUBLIC ${OPTIMIZATION_LIBRARIES})
