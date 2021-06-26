#ifndef LOOP_H
#define LOOP_H

#include <chrono>
#include <iostream>
#include <memory>

#include "robot_vars.h"

// GNC
#include "kleiber_control/ramsete.h"
#include "kleiber_control/point_shoot.h"
#include "kleiber_control/simple_drive_controller.h"
#include "kleiber_guidance/routine.h"
#include "kleiber_guidance/trajectory_reader.h"
#include "kleiber_guidance/vector_action.h"
#include "kleiber_navigation/diffdrive_ekf.h"

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

        // Actions / Routines
        std::shared_ptr<Routine> routine_;

        // Trajectory reader
        std::shared_ptr<TrajectoryReader> traj_reader_;

        // Controller
        std::shared_ptr<SimpleDriveController> controller_;

        // EKF
        std::shared_ptr<DiffDriveEKF> robot_ekf_;

        // Timing
        double loop_time_;
        std::chrono::steady_clock::time_point last_time_;
};

#endif