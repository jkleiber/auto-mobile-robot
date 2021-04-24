#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>

typedef struct traj_pt_t {
    Eigen::Vector3d state;
    double linear_velocity;
    double angular_velocity;
    Eigen::Vector2d lin_vel_constraints;
    Eigen::Vector2d ang_vel_constraints;
} TrajectoryPoint;

#endif