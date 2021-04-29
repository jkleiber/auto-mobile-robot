#! /bin/bash

# Start gazebo
./simulation/six-wheel-robot/launch_slalom_world.sh &

# Change to the build directory
cd build

# Start sim robot code
./src/auto_mobile_robot_sim
