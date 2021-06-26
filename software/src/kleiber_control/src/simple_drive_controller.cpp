#include "kleiber_control/simple_drive_controller.h"

SimpleDriveController::~SimpleDriveController(){}

void SimpleDriveController::init()
{
    // Initialize the controller
    yaw_control_.init();
    yaw_control_.set_wrap(true, -M_PI, M_PI);
}

void SimpleDriveController::update()
{
    // If the command is disabled, output 0.
    if(!cmd_->enabled)
    {
        ctrl_->u(0) = 0.0;
        ctrl_->u(1) = 0.0;
        return;
    }

    // Get the robot state
    yaw_ = state_->x(2);

    // Get the setpoints commanded
    yaw_setpoint_ = cmd_->yaw;
    double vel_setpoint = cmd_->vel;

    // Find the turn power via a PID to control turning to the correct yaw
    yaw_control_.update();

    // Send the output
    ctrl_->u(0) = vel_setpoint;
    ctrl_->u(1) = w_out_;

    // std::cout << yaw_control_.get_error() << "   " << w_out_ << std::endl;
}