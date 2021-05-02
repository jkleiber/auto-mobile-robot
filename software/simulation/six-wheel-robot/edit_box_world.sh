#! /bin/bash

cd "$(dirname $0)"

export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/models


gazebo worlds/basic-box-world.sdf