#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include "GolfBallFlight.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "math_utils.hpp"

#include <vector>

class Simulator
{
public:
    Simulator(GolfBallFlight &flight);

    auto runSimulation() -> std::vector<Vector3D>;
    auto runSimulationLanding() -> Vector3D;

private:
    GolfBallFlight &flight;

    float currentTime = 0.0F;

    [[nodiscard]] auto isFlightOver() const -> bool;
    void calculateNextStep();
};

#endif // SIMULATOR_HPP
