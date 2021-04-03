#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <Eigen/Dense>

typedef struct robot_state_t {
    Eigen::Vector3d x;
} RobotState;


#endif
