#include "kleiber_guidance/routine.h"

Routine::~Routine() {}

void Routine::update()
{
    // If the active action is stopped and had been started before, then increment the action index
    if (current_action_->is_stopped() && current_action_->is_started())
    {
        action_index_ += 1;
    }

    // Do not continue if the actions have all been completed.
    if (action_index_ >= actions_.size())
    {
        return;
    }

    // Update the current action
    current_action_ = actions_.at(action_index_);

    // If the current action hasn't been started yet, then start it
    if (!current_action_->is_started())
    {
        std::cout << "Started action #" << action_index_ + 1 << std::endl;
        current_action_->start();
    }

    // Run the current action update function
    current_action_->update();
}