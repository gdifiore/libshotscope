#include "Simulator.hpp"

#include "GolfBallPhysicsVariables.hpp"
#include "GolfBallFlight.hpp"
#include "golf_ball.hpp"
#include "math_utils.hpp"

Simulator::Simulator(GolfBallPhysicsVariables& physVars, GolfBallFlight& flight)
    : physVars(physVars), flight(flight)
{

}

Vector3D Simulator::runSimulation() {
    int maxIterations = 1000; // Set the maximum number of iterations
    int iteration = 0; // Initialize the iteration counter
    Vector3D lastPosition = flight.getPosition(); // Store the last position

    while (!isFlightOver() && iteration < maxIterations) {
        calculateNextStep();
        currentTime += dt;
        iteration++;

        if (flight.getPosition()[2] < 0) {
            break; // Exit the loop if flight goes below the z-axis
        }

        lastPosition = flight.getPosition(); // Update the last position
    }

    return lastPosition;
}

void Simulator::calculateNextStep() {
    flight.calculateFlightStep();
}

bool Simulator::isFlightOver() const {
    return flight.getPosition()[2] < 0;
}
