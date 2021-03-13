# AMR Firmware

Currently all-in on arduino. Eventually I will upgrade this to STM Nucleo at least, and potentially even a custom board.

## Setup
1. Install the Arduino IDE on the Jetson Nano  
2. Start the X server on the Jetson Nano with `startx`
3. On your local computer, run `ssh -X justin@kleiber-robot -C arduino` to get the Arduino GUI to appear on your screen
4. Ensure the required libraries are installed:
   * RobotLib
   * Adafruit Unified Sensor 
   * Adafruit BNO055
   * ArduinoJson
   * StreamUtils 

Ideally step 4 can be completed using the firmware-setup.sh script in the top level directory, but that doesn't work yet.

Also, steps 2-3 are used to deploy code to the arduino from the Jetson.