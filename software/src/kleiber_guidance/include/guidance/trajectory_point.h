#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>

struct TrajectoryPoint {
    TrajectoryPoint(){}
    TrajectoryPoint(double _t, 
                    Eigen::Vector3d _state, 
                    double v, 
                    double w, 
                    Eigen::Vector2d vlim, 
                    Eigen::Vector2d wlim) : 
                t(_t),
                state(_state),
                linear_velocity(v),
                angular_velocity(w),
                lin_vel_constraints(vlim),
                ang_vel_constraints(wlim) {}
    double t;
    Eigen::Vector3d state;
    double linear_velocity;
    double angular_velocity;
    Eigen::Vector2d lin_vel_constraints;
    Eigen::Vector2d ang_vel_constraints;

    TrajectoryPoint operator* (double factor) const
    {
        return TrajectoryPoint(t*factor, 
            state*factor, 
            linear_velocity*factor, 
            angular_velocity*factor,
            lin_vel_constraints*factor,
            ang_vel_constraints*factor);
    }

    TrajectoryPoint operator+ (const TrajectoryPoint &traj_pt)
    {
        return TrajectoryPoint(t + traj_pt.t, 
            state + traj_pt.state, 
            linear_velocity + traj_pt.linear_velocity, 
            angular_velocity + traj_pt.angular_velocity,
            lin_vel_constraints + traj_pt.lin_vel_constraints,
            ang_vel_constraints + traj_pt.ang_vel_constraints);
    }
};

#endif