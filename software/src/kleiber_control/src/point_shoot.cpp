#include "kleiber_control/point_shoot.h"

PointShootControl::~PointShootControl(){}

void PointShootControl::init()
{
    w_ctrl_.init();
    w_ctrl_.set_wrap(true, -M_PI, M_PI);
}

void PointShootControl::update()
{
    // Get the angle
    angle_ = state_->x(2);

    // Sit still until we are lined up.
    double v_out = 0.0;
    double angle_err = fabs(w_ctrl_.get_error());
    if (angle_err < (3*M_PI/180.0))
    {
        v_out = 0.6;
    }
    else if (angle_err < (15*M_PI / 180.0))
    {
        v_out = 0.25;
    }
    else 
    {
        v_out = 0.05;
    }

    v_out = v_out * 5;
    
    // Aim
    double dx = ref_traj_->state(0) - state_->x(0);
    double dy = ref_traj_->state(1) - state_->x(1);
    angle_ref_ = atan2(dy, dx);

    std::cout << "x: " << dx << " y: " << dy << std::endl;

    // Update the PID
    w_ctrl_.update();

    // Send the output
    ctrl_->u(0) = ControlUtils::constrain(v_out, ref_traj_->lin_vel_constraints(0), ref_traj_->lin_vel_constraints(1));
    ctrl_->u(1) = ControlUtils::constrain(w_out_, ref_traj_->ang_vel_constraints(0), ref_traj_->ang_vel_constraints(1));
}