#ifndef ROBOT_VARIABLES_H
#define ROBOT_VARIABLES_H

// Drivers
#include "drivers/sensor_data.h"

// GNC
#include "gnc/control/control_output.h"
#include "gnc/guidance/trajectory_point.h"
#include "gnc/navigation/robot_state.h"

typedef struct robot_vars_t {
    RamseteOutput ctrl_out;
    TrajectoryPoint ref_traj;
    RobotState robot_state;
    SensorData sensor_input;
} RobotVariables;

#endif