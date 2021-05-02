#include "control/utils.h"


double ControlUtils::constrain(double x, double min_x, double max_x)
{
    return std::fmin(max_x, std::fmax(x, min_x));
}