#include "GolfBallKinematics.hpp"

void GolfBallKinematics::calcTime()
{
    t += dt;
}

void GolfBallKinematics::calculateAllVariables()
{
    // this means the ball sunk below the ground and we have issues
    while (z > 0)
    {
    }
}