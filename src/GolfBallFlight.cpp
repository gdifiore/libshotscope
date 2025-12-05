/**
 * @file GolfBallFlight.cpp
 * @author Gabriel DiFiore
 * @brief Contains the implementation of the GolfBallFlight class.
 *
 * This file defines the GolfBallFlight class, which is responsible for calculating
 * the acceleration, velocity, and other required variables for calculating the
 * final position of a hit golf ball.
 *
 * The GolfBallPhysicsVariables class takes a GolfBallPhysicsVariables object, golf ball object, and atmospheric data as input,
 * and provides methods to calculate all the required variables.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#include "GolfBallFlight.hpp"
#include "FlightPhase.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <memory>

/**
 * @brief Constructs a GolfBallFlight object.
 *
 * This constructor initializes a GolfBallFlight object with the given physics variables,
 * golf ball data, and atmospheric data. It sets up the initial state for simulating
 * the flight of a golf ball.
 *
 * @param physicsVars Reference to a GolfBallPhysicsVariables object containing pre-calculated physics variables.
 * @param ball The golf ball structure containing relevant data such as initial position, velocity, and spin.
 * @param atmos The atmospheric data structure containing relevant environmental data.
 *
 * @note The user is responsible for validating the physicsVars, ball, and atmos parameters before passing them to this constructor.
 *       This class assumes that the input data is valid, consistent, and within physically reasonable ranges.
 *       Passing invalid, inconsistent, or out-of-range data may lead to unexpected behavior or incorrect flight calculations.
 *
 * @note The physicsVars object should be initialized with the same ball and atmos data used in this constructor
 *       to ensure consistency in calculations.
 */
GolfBallFlight::GolfBallFlight(
	GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
	const struct atmosphericData &atmos) : physicsVars(physicsVars), ball(ball), atmos(atmos)
{

	initialize();
}

/**
 * Initializes the golf ball flight by setting the initial position, velocity, and creating the aerial phase.
 *
 * Some of these have initial values from other classes, some need to be calculated for the first time.
 */
void GolfBallFlight::initialize()
{
	state.position = {ball.x0, ball.y0, ball.z0};
	state.currentTime = 0.0F;
	state.velocity = physicsVars.getV0Vector();
	state.acceleration = {0.0F, 0.0F, 0.0F};

	// Create the aerial phase
	aerialPhase = std::make_unique<AerialPhase>(physicsVars, ball, atmos);

	// Initialize the phase with the current state to calculate initial values
	aerialPhase->initialize(state);
}

float GolfBallFlight::determineCoefficientOfDrag()
{
	return aerialPhase->determineCoefficientOfDrag();
}

float GolfBallFlight::determineCoefficientOfLift()
{
	return aerialPhase->determineCoefficientOfLift();
}

/**
 * Calculates a single step in the golf ball's flight.
 * Delegates to the current flight phase (AerialPhase).
 */
void GolfBallFlight::calculateFlightStep()
{
	aerialPhase->calculateStep(state, physics_constants::SIMULATION_TIME_STEP);
}