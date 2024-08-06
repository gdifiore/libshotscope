#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <vector>

#include "GolfBallFlight.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "math_utils.hpp"

class Simulator
{
public:
    Simulator(GolfBallFlight &flight);

    auto runSimulation() -> std::vector<Vector3D>;
    auto runSimulationLanding() -> Vector3D;

private:
    GolfBallFlight &flight;

    const float dt = 0.01F;
    float currentTime = 0.0F;

    [[nodiscard]] auto isFlightOver() const -> bool;
    void calculateNextStep();
};;

#endif // SIMULATOR_HPP
