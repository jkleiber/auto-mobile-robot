#ifndef TRAJECTORY_READER_H
#define TRAJECTORY_READER_H

// System
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <deque>
#include <filesystem>

// Third Party
#include <Eigen/Dense>
#include <Eigen/Core>

// GNC
#include "trajectory_point.h"
#include "kleiber_navigation/robot_state.h"


class TrajectoryReader {

    public:
        TrajectoryReader(std::string filename, 
                         TrajectoryPoint *ref_traj,
                         RobotState *robot_x,
                         double *t) 
            : filename_(filename),
              ref_pt_(ref_traj),
              cur_state_(robot_x),
              t_ref_(t) {}

        void init();
        void update();

        virtual ~TrajectoryReader();

    private:
        // Interfacing with the loop
        std::string filename_;
        TrajectoryPoint *ref_pt_;
        RobotState *const cur_state_;
        double *const t_ref_;

        // Keep track of the trajectory
        std::deque<TrajectoryPoint> traj_;

        // File reading
        std::ifstream traj_file_;

        int active_traj_pt_;

};


#endif