#ifndef RAMSETE_CONTROLLER_H
#define RAMSETE_CONTROLLER_H

// System includes
#include <memory>

// Third party includes
#include <Eigen/Dense>

// Control library includes
#include "controller.h"
#include "control_output.h"

// Other GNC includes
#include "guidance/trajectory_point.h"
#include "navigation/robot_state.h"

class RamseteController : public Controller {
    public:
        RamseteController(TrajectoryPoint *ref_traj, 
                          RobotState *state, 
                          RamseteOutput *ctrl_out,
                          double zeta, 
                          double b) :
            ref_traj_(ref_traj), 
            state_(state), 
            ctrl_out_(ctrl_out),
            zeta_(zeta), 
            b_(b) {}

        void output();

        virtual ~RamseteController();
    
    private:
        // Track setpoint and state via pointers
        TrajectoryPoint *const ref_traj_;
        RobotState *const state_;

        // Controller output
        RamseteOutput *ctrl_out_;
        
        // Controller gain parameters
        double zeta_;
        double b_;

        // Control calculations
        double sinc(double x);
        double constrain(double x, double min_x, double max_x);
        
};

#endif