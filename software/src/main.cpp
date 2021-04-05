
#include <iostream>
#include <memory>

#include "robot_vars.h"
#include "drivers/arduino_interface.h"
#include "gnc/control/ramsete.h"


int main(int argc, char **argv) {
    
    // Initialize the robot variables shared memory
    std::shared_ptr<RobotVariables> robot_vars = std::make_shared<RobotVariables>();

    std::shared_ptr<RamseteController> controller = 
        std::make_shared<RamseteController>(
            &robot_vars->ref_traj, 
            &robot_vars->robot_state, 
            &robot_vars->ctrl_out, 
            0.4, 
            18);

    controller->output();

    std::shared_ptr<ArduinoInterface> arduino_interface = 
        std::make_shared<ArduinoInterface>(
            &robot_vars->ctrl_out, 
            &robot_vars->sensor_input, 
            "/dev/ttyUSB0");


    for (int i = 0; i < 100; ++i)
    {
        std::cout << i << std::endl;
        arduino_interface->update();
    }

    return 0;
}