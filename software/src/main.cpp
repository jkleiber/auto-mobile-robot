
#include <iostream>
#include <memory>

#include "robot_vars.h"
#include "arduino_interface.h"

// GNC
#include "control/ramsete.h"
#include "navigation/diffdrive_ekf.h"


int main(int argc, char **argv) {
    
    // Initialize the robot variables shared memory
    std::shared_ptr<RobotVariables> robot_vars = std::make_shared<RobotVariables>();

    // Robot controller
    std::shared_ptr<RamseteController> controller = 
        std::make_shared<RamseteController>(
            &robot_vars->ref_traj, 
            &robot_vars->robot_state, 
            &robot_vars->ctrl_out, 
            0.4, 
            18);

    // Navigation
    std::shared_ptr<DiffDriveEKF> robot_ekf = 
        std::make_shared<DiffDriveEKF>(
            &robot_vars->ekf_input,
            &robot_vars->ctrl_out, 
            &robot_vars->robot_state
        );
    robot_ekf->init();

    // Arduino bridge
    std::shared_ptr<ArduinoInterface> arduino_interface = 
        std::make_shared<ArduinoInterface>(
            &robot_vars->ctrl_out, 
            &robot_vars->ekf_input, 
            "/dev/ttyUSB0");

    controller->output();

    for (int i = 0; i < 100; ++i)
    {
        std::cout << i << std::endl;
        arduino_interface->update();

        robot_ekf->update();
    }

    return 0;
}