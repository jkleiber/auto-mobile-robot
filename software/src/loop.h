#ifndef LOOP_H
#define LOOP_H

#include <memory>

#include "robot_vars.h"

// GNC
#include "control/ramsete.h"
#include "navigation/diffdrive_ekf.h"

class Loop {
    public:
        Loop(RobotVariables *robot_vars) :
            robot_vars_(robot_vars){}

        void init();
        void update();

        virtual ~Loop();

    private:
        // Robot variables
        RobotVariables *robot_vars_;

        // Controller
        std::shared_ptr<RamseteController> controller_;

        std::shared_ptr<DiffDriveEKF> robot_ekf_;
};

#endif