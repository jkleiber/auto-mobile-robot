#ifndef RAMSETE_CONTROLLER_H
#define RAMSETE_CONTROLLER_H

// System includes
#include <iostream>
#include <memory>

// Third party includes
#include <Eigen/Dense>

// Control library includes
#include "controller.h"
#include "control_output.h"
#include "utils.h"

// Other GNC includes
#include "guidance/trajectory_point.h"
#include "navigation/robot_state.h"

class RamseteController : public Controller {
    public:
        RamseteController(TrajectoryPoint *ref_traj, 
                          RobotState *state, 
                          VelocityControl *ctrl_out,
                          double zeta, 
                          double b) :
            ref_traj_(ref_traj), 
            state_(state), 
            ctrl_out_(ctrl_out),
            zeta_(zeta), 
            b_(b) {}

        void update();

        virtual ~RamseteController();
    
    private:
        // Track setpoint and state via pointers
        TrajectoryPoint *const ref_traj_;
        RobotState *const state_;

        // Controller output
        VelocityControl *ctrl_out_;
        
        // Controller gain parameters
        double zeta_;
        double b_;

        // Control calculations
        double sinc(double x);
        
};

#endif