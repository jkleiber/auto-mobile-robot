#ifndef VECTOR_ACTION_H
#define VECTOR_ACTION_H

#include <iostream>

#include "kleiber_guidance/action.h"
#include "kleiber_guidance/simple_command.h"

class VectorAction : public Action
{
    public:
        VectorAction(double duration, double yaw, double linear_velocity, SimpleCommand *cmd) : 
            Action(duration), yaw_(yaw), linear_velocity_(linear_velocity), cmd_(cmd) {}
        
        void start();
        void run();
        void stop();

        virtual ~VectorAction();
    
    private:
        SimpleCommand *cmd_;

        double yaw_;
        double linear_velocity_;

};

#endif