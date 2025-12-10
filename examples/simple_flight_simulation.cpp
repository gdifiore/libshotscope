/**
 * @file simple_flight_simulation.cpp
 * @brief Demonstrates simplified flight simulation using FlightSimulator class
 *
 * This example shows how FlightSimulator automates phase transitions,
 * making it much easier to simulate complete golf shots.
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include "FlightSimulator.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "physics_constants.hpp"

int main()
{
	std::cout << "=== Simple Flight Simulation with FlightSimulator ===\n\n";

	// Setup ball parameters - typical 7 iron shot
	const golfBall ball{
		0.0F,    // x0
		0.0F,    // y0
		0.0F,    // z0
		135.0F,  // exitSpeed (mph)
		15.0F,   // launchAngle (degrees)
		0.0F,    // direction (degrees)
		5500.0F, // backspin (rpm)
		0.0F     // sidespin (rpm)
	};

	// Standard atmospheric conditions
	const atmosphericData atmos{
		70.0F,  // temperature (F)
		0.0F,   // wind x
		0.0F,   // wind y
		0.0F,   // wind z
		0.0F,   // wind direction
		50.0F,  // humidity
		29.92F  // pressure
	};

	// Fairway ground conditions
	GroundSurface ground;
	ground.height = 0.0F;
	ground.restitution = 0.35F;
	ground.frictionStatic = 0.6F;
	ground.frictionDynamic = 0.2F;
	ground.firmness = 0.7F;

	std::cout << "Shot parameters:\n";
	std::cout << "  Ball speed: " << ball.exitSpeed << " mph\n";
	std::cout << "  Launch angle: " << ball.launchAngle << "°\n";
	std::cout << "  Backspin: " << ball.backspin << " rpm\n\n";

	// Initialize physics and simulator
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Setup initial state - uses sensible defaults for position, acceleration, and time
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	BallState initialState = BallState::fromLaunchParameters(
		v0_fps,
		ball.launchAngle,
		ball.direction
	);

	// Initialize the simulator
	sim.initialize(initialState);

	// Track statistics
	float maxHeight = 0.0F;
	std::string lastPhase = "";
	int phaseTransitions = 0;

	// Run simulation - notice how simple this is!
	const float dt = 0.01F;
	std::cout << "Running simulation...\n";

	while (!sim.isComplete())
	{
		// Just step - FlightSimulator handles all phase transitions automatically
		sim.step(dt);

		// Track phase transitions for reporting
		std::string currentPhase = sim.getCurrentPhaseName();
		if (currentPhase != lastPhase && !lastPhase.empty())
		{
			std::cout << "  Transitioned to " << currentPhase << " phase at t="
			          << std::fixed << std::setprecision(2)
			          << sim.getState().currentTime << "s\n";
			phaseTransitions++;
		}
		lastPhase = currentPhase;

		// Track max height
		maxHeight = std::max(maxHeight, sim.getState().position[2]);
	}

	// Calculate final statistics
	const BallState& finalState = sim.getState();
	float totalDistance = std::sqrt(
		finalState.position[0] * finalState.position[0] +
		finalState.position[1] * finalState.position[1]
	) / physics_constants::YARDS_TO_FEET;

	std::cout << "\nSimulation complete!\n\n";
	std::cout << "Flight Statistics:\n";
	std::cout << "  Total distance: " << std::fixed << std::setprecision(1)
	          << totalDistance << " yards\n";
	std::cout << "  Max height: " << maxHeight << " feet\n";
	std::cout << "  Flight time: " << std::setprecision(2)
	          << finalState.currentTime << " seconds\n";
	std::cout << "  Phase transitions: " << phaseTransitions << "\n\n";

	std::cout << "Final position:\n";
	std::cout << "  x: " << std::setprecision(2) << finalState.position[0] << " ft\n";
	std::cout << "  y: " << finalState.position[1] << " ft\n";
	std::cout << "  z: " << finalState.position[2] << " ft\n\n";

	std::cout << "Notice how simple this code is compared to manually\n";
	std::cout << "managing phase transitions! FlightSimulator handles\n";
	std::cout << "all the complexity automatically.\n";

	return 0;
}
