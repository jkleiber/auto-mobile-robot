
#include <iostream>
#include <memory>

#include "robot_vars.h"
#include "arduino_interface.h"
#include "loop.h"



int main(int argc, char **argv) {
    
    // Initialize the robot variables shared memory
    std::shared_ptr<RobotVariables> robot_vars = std::make_shared<RobotVariables>();

    std::shared_ptr<Loop> robot_loop = std::make_shared<Loop>(robot_vars.get());
    robot_loop->init();

    // Arduino bridge
    std::shared_ptr<ArduinoInterface> arduino_interface = 
        std::make_shared<ArduinoInterface>(
            &robot_vars->ctrl_out, 
            &robot_vars->ekf_input, 
            "/dev/ttyUSB0");

    while(true)
    {
        robot_loop->update();

        arduino_interface->update();
    }

    return 0;
}