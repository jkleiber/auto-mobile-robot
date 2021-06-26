#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <chrono>
#include <algorithm>
#include <iostream>
#include <math.h>

#include "controller.h"

class PIDController
{
    public:
        PIDController(double kp, double ki, double kd, double *setpoint, double *state, double *output) :
            Kp_(kp), Ki_(ki), Kd_(kd), setpoint_(setpoint), state_(state), output_(output) {}

        void init();
        void update();

        void gains(double kp, double ki, double kd);
        void set_wrap(bool is_wrap, double x_min, double x_max);

        double get_error();

        virtual ~PIDController();

    private:
        // Gains
        double Kp_;
        double Ki_;
        double Kd_;

        // Reference, state, output pointers
        double *const setpoint_;
        double *const state_;
        double *output_;

        // PID stuff
        double integrator_;
        double last_state_;
        double last_error_;

        // Handle wrapped states
        bool is_wrap_;
        double wrap_min_;
        double wrap_max_;
        double wrap_range_;

        // Timing
        double dt_;
        std::chrono::steady_clock::time_point last_time_;
};

#endif