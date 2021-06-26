#include "kleiber_guidance/action.h"

Action::~Action(){}

void Action::update()
{
    // If timeout is negative then keep running until the action is completed.
    if (timeout_ < 0)
    {
        return;
    }

    // Find the elapsed time.
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    double elapsed_time = std::chrono::duration<double>(current_time - start_time_).count();

    // If the elapsed time is less than the action timeout, then run the run() function.
    if(elapsed_time < timeout_ && !is_stopped_ && is_started_)
    {
        this->run();
    }
    // Otherwise stop the action.
    else
    {
        this->stop();
    }
}

void Action::stop_action()
{
    // Stop the action
    is_stopped_ = true;
}

void Action::start_action()
{
    // Start the action and timer
    is_started_ = true;
    start_time_ = std::chrono::steady_clock::now();
}

bool Action::is_started()
{
    return is_started_;
}

bool Action::is_stopped()
{
    return is_stopped_;
}