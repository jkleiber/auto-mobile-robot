% Control gains
    zeta = 0.4;
    b = 18;
    K = 2*zeta*sqrt(w_in^2 + b*v_in^2);
    k1 = K;
    k2 = b;
    k3 = K;

    % Find error in robot frame
    delta = state - x;
    theta = x(3);
    R = [cos(theta) sin(theta) 0;
        -sin(theta) cos(theta) 0;
             0          0      1];
    E = R*delta;
    ex = E(1);
    ey = E(2);
    et = E(3);
    
    % Nonlinear control (Ramsete Controller)
    U = [-k1 * ex;
         -k2 * v_in * sinc(et) * ey - k3 * et];
    
    % Command to drivetrain after nonlinear transform
    u = [v_in*cos(et) - U(1);
         w_in - U(2)];
     
    u(1) = constrain(u(1), 0, 6);
    u(2) = constrain(u(2), -6, 6);

#include "ramsete.h"

Eigen::Vector2d RamseteController::output(Eigen::Vector3d setpoint, Eigen::Vector3d x, double lin_vel, double ang_vel)
{
    // Helper variables
    double theta = x(2);

    // Find difference in robot state and setpoint
    Eigen::Vector3d delta = setpoint - x;

    // Get rotation matrix from robot frame to global frame
    Eigen::Rotation2D<double> R(theta);

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

    // TODO: constrain the output

    // Form the output
    Eigen::Vector2d control_output(v_cmd, w_cmd);

    return control_output;
}


double RamseteController::sinc(double x)
{
    if (x == 0.0)
    {
        return 0;
    }

    return (sin(x) / x);
}


double RamseteController::constrain(double x, double min_x, double max_x)
{
    return std::fmin(max_x, std::fmax(x, min_x));
}