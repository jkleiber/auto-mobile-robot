#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

// System
#include <chrono>

// Third party includes
#include <Eigen/Dense>

// Robot
#include "gnc/navigation/robot_state.h"
#include "gnc/control/control_output.h"
#include "drivers/sensor_data.h"

class DiffDriveEKF {
    public:
        DiffDriveEKF(SensorData *sensor_data,
            RamseteOutput *ctrl,
            RobotState *robot_state) : 
                sensor_input_(sensor_data),
                ctrl_(ctrl),
                robot_state_(robot_state) {}

        void init();
        void update();

        virtual ~DiffDriveEKF();

    private:
        // Sensor and control data
        SensorData *const sensor_input_;
        RamseteOutput *const ctrl_;
        
        // 3DOF state predictions
        RobotState *robot_state_;

        // Full state prediction
        Eigen::VectorXd ekf_state_;

        // EKF matrices
        Eigen::MatrixXd P_, S_, H_, F_, K_, I_;
        Eigen::MatrixXd Q_, R_;

        // Timing
        std::chrono::steady_clock::time_point last_time_;


        // Robot dynamics and jacobian
        Eigen::VectorXd dynamics(Eigen::VectorXd x, Eigen::Vector2d u, double dt);
        void dynamics_jacobian(Eigen::VectorXd x, Eigen::Vector2d u, double dt);

        // Measurement model and jacobian
        Eigen::VectorXd measurement_model(Eigen::VectorXd x);
        void measurement_jacobian(Eigen::VectorXd x);
};


#endif
