#ifndef RAMSETE_CONTROLLER_H
#define RAMSETE_CONTROLLER_H

#include <Eigen/Dense>
#include "controller.h"

class RamseteController : public Controller {
    public:
        RamseteController(double zeta, double b) :
             zeta_(zeta), b_(b) {}

        Eigen::Vector2d output(Eigen::VectorXd setpoint, Eigen::VectorXd state, double lin_vel, double ang_vel);
    
    private:
        double zeta_;
        double b_;

        double sinc(double x);
        double constrain(double x, double min_x, double max_x);
        
};

#endif