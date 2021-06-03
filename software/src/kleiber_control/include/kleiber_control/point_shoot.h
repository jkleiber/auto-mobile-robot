#ifndef POINT_SHOOT_H
#define POINT_SHOOT_H

// System includes
#include <iostream>
#include <memory>

// Third party includes
#include <Eigen/Dense>

// Control library includes
#include "controller.h"
#include "control_output.h"
#include "pid_controller.h"
#include "utils.h"

// Other GNC includes
#include "kleiber_guidance/trajectory_point.h"
#include "kleiber_navigation/robot_state.h"

class PointShootControl : public Controller
{
    public:
        PointShootControl(TrajectoryPoint *ref_traj, 
                          RobotState *state, 
                          VelocityControl *ctrl_out) :
                ref_traj_(ref_traj),
                state_(state),
                ctrl_(ctrl_out),
                w_ctrl_(5.0, 0.0, 0.0, &angle_ref_, &angle_, &w_out_) {}

        void init();
        void update();

        virtual ~PointShootControl();
    
    private:
        // Track setpoint and state via pointers
        TrajectoryPoint *const ref_traj_;
        RobotState *const state_;

        VelocityControl *ctrl_;

        // Velocity setpoints
        double angle_ref_;
        double angle_;
        double w_out_;

        // PID Controllers
        PIDController w_ctrl_;
};

#endif