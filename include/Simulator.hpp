#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "GolfBallFlight.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "math_utils.hpp"

class Simulator
{
public:
    Simulator(GolfBallPhysicsVariables& physVars, GolfBallFlight& flight);

    Vector3D runSimulation();

private:
    GolfBallPhysicsVariables& physVars;
    GolfBallFlight& flight;

    const float dt = 0.01f;
    float currentTime = 0.0f;

    bool isFlightOver() const;
    void calculateNextStep();
};

#endif // SIMULATOR_HPP
