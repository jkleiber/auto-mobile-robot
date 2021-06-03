#ifndef CONTROLLER_OUTPUT_H
#define CONTROLLER_OUTPUT_H

#include <Eigen/Dense>

typedef struct velocity_ctrl_output_t {
    Eigen::Vector2d u;
} VelocityControl;

#endif