#ifndef ROBOT_VARIABLES_H
#define ROBOT_VARIABLES_H

// GNC
#include "kleiber_control/control_output.h"
#include "kleiber_guidance/trajectory_point.h"
#include "kleiber_navigation/robot_state.h"
#include "kleiber_navigation/diffdrive_ekf.h"
#include "kleiber_guidance/simple_command.h"

typedef struct robot_vars_t {
    VelocityControl ctrl_out;
    TrajectoryPoint ref_traj;
    RobotState robot_state;
    DiffDriveEKFInput ekf_input;
    SimpleCommand simple_cmd;
} RobotVariables;

#endif