#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H


// System
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <deque>
#include <filesystem>

// Third Party
#include <Eigen/Dense>

// GNC
#include "trajectory_point.h"
#include "kleiber_navigation/robot_state.h"



class PurePursuit {

    public:
        PurePursuit(std::string filename, 
                         TrajectoryPoint *ref_traj,
                         RobotState *robot_x,
                         double *t) 
            : filename_(filename),
              ref_pt_(ref_traj),
              cur_state_(robot_x),
              t_ref_(t),
              closest_pt_idx_(0) {}

        void init();
        void update();

        virtual ~PurePursuit();

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

        // Make sure there is a lower bound on lookahead index
        int closest_pt_idx_;

};


#endif