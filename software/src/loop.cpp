#include "loop.h"

Loop::~Loop(){}

void Loop::init()
{
    // Robot controller
    controller_ = std::make_shared<RamseteController>(
                    &robot_vars_->ref_traj, 
                    &robot_vars_->robot_state, 
                    &robot_vars_->ctrl_out, 
                    0.4, 
                    18
                );

    // Navigation
    robot_ekf_ = std::make_shared<DiffDriveEKF>(
            &robot_vars_->ekf_input,
            &robot_vars_->ctrl_out, 
            &robot_vars_->robot_state
        );
    robot_ekf_->init();
}

void Loop::update()
{
    // Update EKF
    robot_ekf_->update();

    // Get Controller output
    controller_->output();

    // Test for now just a slow turning arc
    robot_vars_->ctrl_out.u(0) = 0;
    robot_vars_->ctrl_out.u(1) = 0.2;
}