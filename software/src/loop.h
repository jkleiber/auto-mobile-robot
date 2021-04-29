#ifndef LOOP_H
#define LOOP_H

#include <chrono>
#include <iostream>
#include <memory>

#include "robot_vars.h"

// GNC
#include "control/ramsete.h"
#include "guidance/trajectory_reader.h"
#include "navigation/diffdrive_ekf.h"

class Loop {
    public:
        Loop(RobotVariables *robot_vars) :
            robot_vars_(robot_vars){}

        void init();
        void update();

        void reset_timing();

        virtual ~Loop();

    private:
        // Robot variables
        RobotVariables *robot_vars_;

        // Trajectory reader
        std::shared_ptr<TrajectoryReader> traj_reader_;

        // Controller
        std::shared_ptr<RamseteController> controller_;

        // EKF
        std::shared_ptr<DiffDriveEKF> robot_ekf_;

        // Timing
        double loop_time_;
        std::chrono::steady_clock::time_point last_time_;
};

#endif