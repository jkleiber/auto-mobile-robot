#include "kleiber_control/pid_controller.h"

PIDController::~PIDController(){}

void PIDController::init()
{
    last_state_ = *state_;
    integrator_ = 0.0;
    last_error_ = *setpoint_ - *state_;
}


void PIDController::update()
{
    // Calculate error based on state wrapping / no wrapping
    double error = get_error();
    
    // Proportional component
    double P = this->Kp_ * error;

    // Error curve
    double d_err = error - last_error_;

    // Compute elapsed time in seconds
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    dt_ = std::chrono::duration<double>(current_time - last_time_).count();
    last_time_ = current_time;

    // Derivative
    double D = this->Kd_ * (d_err / dt_);

    // Integration (trapezoidal)
    double slice = dt_ * (last_error_ + error) / 2.0;
    integrator_ += slice;
    double I = this->Ki_ * integrator_;

    // Compute the output
    *output_ = P + I + D;

    // Update the error tracker
    last_error_ = error;
}

void PIDController::gains(double kp, double ki, double kd)
{
    this->Kp_ = kp;
    this->Ki_ = ki;
    this->Kd_ = kd;
}

void PIDController::set_wrap(bool is_wrap, double x_min, double x_max)
{
    this->is_wrap_ = is_wrap;
    this->wrap_min_ = x_min;
    this->wrap_max_ = x_max;
    this->wrap_range_ = x_max - x_min;
}

double PIDController::get_error()
{
    double error = 0.0;

    if (!is_wrap_)
    {
        error = *setpoint_ - *state_;
    }
    // For states that wrap, take the wrapping into account
    else
    {
        /* Wrap the state */
        double wrapped_state = *state_;
        double dist_from_edge = 0.0;
        if(*state_ > wrap_max_)
        {
            dist_from_edge = *state_ - wrap_max_;
            wrapped_state = fmod(dist_from_edge, wrap_range_) + wrap_min_;
        }
        else if(*state_ < wrap_min_)
        {
            dist_from_edge = *state_ - wrap_min_;
            wrapped_state = fmod(dist_from_edge, wrap_range_) + wrap_max_;
        }
        // No "else" required because this is handled by the wrapped state being set to *state.

        double err_1 = *setpoint_ - wrapped_state;

        // Check to see if setpoint is bigger than state to set up the correct wrap directions
        double err_2 = 0.0;
        
        if (*setpoint_ > wrapped_state)
        {
            err_2 = (*setpoint_ - wrap_max_) + (wrap_min_ - wrapped_state);
        }
        else
        {
            err_2 = (wrap_max_ - wrapped_state) + (*setpoint_ - wrap_min_);
        }

        // Error is the minimum magnitude of the two possibilities
        // std::cout << "ref: " << *setpoint_ * 180.0 / M_PI << " ang: " << wrapped_state * 180.0 / M_PI << " err1: " << err_1 << " err2: " << err_2 << std::endl;
        error = err_1;
        if(fabs(err_2) < fabs(err_1))
        {
            error = err_2;
        }
    }

    return error;
}
