#include "kleiber_guidance/vector_action.h"

VectorAction::~VectorAction(){}

void VectorAction::start()
{
    start_action();
}

void VectorAction::run()
{
    // Set the command
    cmd_->vel = linear_velocity_;
    cmd_->yaw = yaw_;

    // Set to enabled
    cmd_->enabled = true;
}

void VectorAction::stop()
{
    // Set to enabled
    cmd_->enabled = false;
}