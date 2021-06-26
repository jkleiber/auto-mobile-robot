#ifndef ROUTINE_H
#define ROUTINE_H

#include <iostream>
#include <memory>
#include <vector>

#include "kleiber_guidance/action.h"

class Routine 
{
    public:
        Routine(std::vector<std::shared_ptr<Action> > actions) : actions_(actions), action_index_(0) 
        {
            // Set the active action to the first action in the vector
            current_action_ = actions_.at(0);
        }

        void update();

        virtual ~Routine();

    private:
        std::vector<std::shared_ptr<Action> > actions_;

        std::shared_ptr<Action> current_action_;

        int action_index_;
};

#endif