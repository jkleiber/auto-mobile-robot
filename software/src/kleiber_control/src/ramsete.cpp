#include "kleiber_control/ramsete.h"

void RamseteController::update()
{
    // Helper variables
    double theta = state_->x(2);
    double lin_vel = ref_traj_->linear_velocity;
    double ang_vel = ref_traj_->angular_velocity;

    // Find difference in robot state and setpoint
    Eigen::Vector3d delta = ref_traj_->state - state_->x;
    // std::cout << "theta: " << theta << "\ndtheta: " << delta(2) << "\ntarget: " << ref_traj_->state(2) << std::endl ;

    // Get rotation matrix from robot frame to global frame
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

    // Rotate the error into the robot's frame
    Eigen::Vector3d error = R * delta;

    // Get components of the error
    double err_x = error(0);
    double err_y = error(1);
    double err_t = error(2);    // Theta

    // Compute feedback gains k1, k2, k3
    double k1 = 2 * zeta_ * sqrt(pow(ang_vel,2) + b_ * pow(lin_vel, 2));
    double k2 = b_;
    double k3 = k1;

    // Nonlinear feedback control laws
    double u1 = -k1 * err_x;
    double u2 = -k2 * lin_vel * sinc(err_t) * err_y - k3 * err_t;

    // Augment the feedback control to control the drivetrain
    double v_cmd = lin_vel * cos(err_t) - u1;
    double w_cmd = ang_vel - u2;

    // Constrain the output and send it to the control output variable
    // TODO: constrain output
    ctrl_out_->u(0) = ControlUtils::constrain(v_cmd, ref_traj_->lin_vel_constraints(0), ref_traj_->lin_vel_constraints(1));
    ctrl_out_->u(1) = ControlUtils::constrain(w_cmd, ref_traj_->ang_vel_constraints(0), ref_traj_->ang_vel_constraints(1));

    std::cout << "ctrl: \n" << ctrl_out_->u << std::endl << std::endl;
}

RamseteController::~RamseteController(){}

double RamseteController::sinc(double x)
{
    if (x == 0.0)
    {
        return 0;
    }

    return (sin(x) / x);
}

