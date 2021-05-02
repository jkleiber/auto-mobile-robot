#include "loop.h"

Loop::~Loop(){}

void Loop::init()
{
    // Trajectory reader
    traj_reader_ = std::make_shared<TrajectoryReader>(
        "config/trajectory.csv",
        &robot_vars_->ref_traj,
        &robot_vars_->robot_state,
        &loop_time_
    );
    traj_reader_->init();

    // Robot controller
    controller_ = std::make_shared<PointShootControl>(
                    &robot_vars_->ref_traj, 
                    &robot_vars_->robot_state, 
                    &robot_vars_->ctrl_out
                );
    controller_->init();
    // controller_ = std::make_shared<RamseteController>(
    //                 &robot_vars_->ref_traj, 
    //                 &robot_vars_->robot_state, 
    //                 &robot_vars_->ctrl_out, 
    //                 0.8, 
    //                 20
    //             );

    // Navigation
    robot_ekf_ = std::make_shared<DiffDriveEKF>(
            &robot_vars_->ekf_input,
            &robot_vars_->ctrl_out, 
            &robot_vars_->robot_state
        );
    robot_ekf_->init();

    // Set the clock to 0.
    this->reset_timing();
}

void Loop::update()
{
    // Get the reference point
    traj_reader_->update();

    // Update EKF
    robot_ekf_->update();

    // Get controller output
    controller_->update();

    // std::cout << "State:\n" << robot_vars_->robot_state.x << std::endl << std::endl;
    // std::cout << robot_vars_->ctrl_out.u << std::endl << std::endl;

    // Compute elapsed time in seconds
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_time - last_time_).count();
    last_time_ = current_time;

    // Update the timing
    loop_time_ += dt;
}

void Loop::reset_timing()
{
    last_time_ = std::chrono::steady_clock::now();
    loop_time_ = 0;
}
