#ifndef ROBOT_VARIABLES_H
#define ROBOT_VARIABLES_H

// GNC
#include "control/control_output.h"
#include "guidance/trajectory_point.h"
#include "navigation/robot_state.h"
#include "navigation/diffdrive_ekf.h"

typedef struct robot_vars_t {
    RamseteOutput ctrl_out;
    TrajectoryPoint ref_traj;
    RobotState robot_state;
    DiffDriveEKFInput ekf_input;
} RobotVariables;

#endif