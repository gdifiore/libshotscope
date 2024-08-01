#include "Simulator.hpp"

#include "GolfBallPhysicsVariables.hpp"
#include "GolfBallFlight.hpp"
#include "golf_ball.hpp"
#include "math_utils.hpp"
#include <vector>

Simulator::Simulator(GolfBallPhysicsVariables &physVars, GolfBallFlight &flight)
    : physVars(physVars), flight(flight)
{
}

/**
 * Runs the flight simulation.
 *
 * @return The landing point (yds) as a Vector3D.
 */
Vector3D Simulator::runSimulationLanding()
{
    int maxIterations = 1000;
    int iteration = 0;
    Vector3D previousPosition = flight.getPosition();
    Vector3D currentPosition = previousPosition;

    while (!isFlightOver() && iteration < maxIterations)
    {
        calculateNextStep();

        previousPosition = currentPosition;
        currentPosition = flight.getPosition();

        currentTime += dt;
        iteration++;
    }

    return math_utils::calcLandingPoint({previousPosition, currentPosition});
}

/**
 * Runs the flight simulation and returns the positions of the flight at each step.
 *
 * @note This contains the point just after the ball goes below z=0. This is used by
 *       math_utils::calcLandingPoint to more accurately determine the final position.
 *
 * @return A vector of Vector3D objects representing the positions of the flight at each step.
 */
std::vector<Vector3D> Simulator::runSimulation()
{
    int maxIterations = 1000;
    int iteration = 0;
    std::vector<Vector3D> positions;
    positions.push_back(flight.getPosition());

    while (!isFlightOver() && iteration < maxIterations)
    {
        calculateNextStep();

        positions.push_back(flight.getPosition());

        currentTime += dt;
        iteration++;
    }

    return positions;
}

/**
 * Calculates the next step in the simulation.
 * This function calls the calculateFlightStep() function of the flight object.
 */
void Simulator::calculateNextStep()
{
    flight.calculateFlightStep();
}

/**
 * Checks if the flight is over.
 *
 * @return true if the ball position.z < 0, false otherwise.
 */
bool Simulator::isFlightOver() const
{
    return flight.getPosition()[2] < 0;
}
