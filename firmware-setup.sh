#!/bin/bash

# Install the arduino IDE to the home directory
wget -O ~/ https://downloads.arduino.cc/arduino-1.8.13-linuxaarch64.tar.xz
tar xf arduino-1.8.13-linuxaarch64.tar.xz
sudo ~/arduino-1.8.13/install.sh
rm -rf arduino-1.8.13-linuxaarch64.tar.xz

# Install XVFB
sudo apt-get install xvfb

# Install Arduino Libraries
git clone git@github.com:adafruit/Adafruit_Sensor.git ~/arduino-1.8.13/libraries/Adafruit_Sensor
git clone git@github.com:SoonerRobotics/RobotLib.git ~/arduino-1.8.13/libraries/RobotLib
