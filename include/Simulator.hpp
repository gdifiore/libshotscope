#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <vector>

#include "GolfBallFlight.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "math_utils.hpp"

class Simulator
{
public:
    Simulator(GolfBallPhysicsVariables &physVars, GolfBallFlight &flight);

    std::vector<Vector3D> runSimulation();
    Vector3D runSimulationLanding();

private:
    GolfBallPhysicsVariables &physVars;
    GolfBallFlight &flight;

    const float dt = 0.01f;
    float currentTime = 0.0f;

    bool isFlightOver() const;
    void calculateNextStep();
};

#endif // SIMULATOR_HPP
