#! /bin/bash

# Find compile script working directory
COMPILE_BASE="$(dirname $0)"
cd $COMPILE_BASE
COMPILE_WD="$(pwd)"

# Change to build directory
cd $COMPILE_WD/build

# CMake then make
cmake .. -DCOMPILE_SIMULATION=0
make

# Copy the current configuration to the binary location
cd $COMPILE_WD
rm -rf $COMPILE_WD/build/config
cp -R $COMPILE_WD/config $COMPILE_WD/build/