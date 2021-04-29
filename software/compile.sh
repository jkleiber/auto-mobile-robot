#! /bin/bash

COMPILE_BASE="$(dirname $0)"

# Change to build directory
cd $COMPILE_BASE/build

# CMake then make
cmake ..
make

# Copy the current configuration to the binary location
cd $COMPILE_BASE
cp -R config/ build/

# Compile the simulator plugins
cd $COMPILE_BASE/simulation/six-wheel-robot/plugins/drivetrain/build
cmake ..
make
