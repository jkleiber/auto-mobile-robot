#include "kleiber_control/state_space.h"

StateSpace::~StateSpace(){}

void StateSpace::update()
{
    Eigen::VectorXd xp1 = (A_ * x_) + (B_ * (*u_));
    *y_ = (C_ * x_) + (D_ * (*u_));

    x_ = xp1;
}