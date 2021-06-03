#ifndef LINEAR_MPC_H
#define LINEAR_MPC_H

#include <nlopt.hpp>

class LinearMPC {
    public:
        LinearMPC();

        void init();
        void update();

        virtual ~LinearMPC();

    private:
        
};

#endif