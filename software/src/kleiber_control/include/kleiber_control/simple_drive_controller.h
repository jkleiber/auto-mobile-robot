#ifndef SIMPLE_DRIVE_CONTROLLER_H
#define SIMPLE_DRIVE_CONTROLLER_H

// Control includes
#include "control_output.h"
#include "pid_controller.h"
#include "utils.h"

// Other GNC includes
#include "kleiber_guidance/simple_command.h"
#include "kleiber_guidance/trajectory_point.h"
#include "kleiber_navigation/robot_state.h"

class SimpleDriveController
{
    public:
        SimpleDriveController(SimpleCommand *cmd,
                              RobotState *state, 
                              VelocityControl *ctrl_out)
            : cmd_(cmd),
              state_(state),
              ctrl_(ctrl_out),
              yaw_control_(3.0, 0, 0, &yaw_setpoint_, &yaw_, &w_out_) {}

        void init();
        void update();

        virtual ~SimpleDriveController();

    private:
        // Command
        SimpleCommand *const cmd_;

        // Robot state
        RobotState *const state_;

        // Control output
        VelocityControl *ctrl_;

        // PID
        PIDController yaw_control_;

        // PID I/O
        double yaw_setpoint_;
        double yaw_;
        double w_out_;
};

#endif