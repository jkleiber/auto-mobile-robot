#! /bin/bash

# Change to build directory
cd build

# CMake then make
cmake ..
make

# Compile the simulator plugins
cd ../simulation/six-wheel-robot/plugins/drivetrain/build
cmake ..
make
