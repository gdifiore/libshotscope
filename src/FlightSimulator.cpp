/**
 * @file FlightSimulator.cpp
 * @author Gabriel DiFiore
 * @brief Implementation of the FlightSimulator class for automated phase management.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#include "FlightSimulator.hpp"
#include <stdexcept>

FlightSimulator::FlightSimulator(
	GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
	const struct atmosphericData &atmos, const GroundSurface &ground)
	: currentPhase(Phase::Aerial), initialized(false),
	  aerialPhase(physicsVars, ball, atmos, ground),
	  bouncePhase(physicsVars, ball, atmos, ground),
	  rollPhase(physicsVars, ball, atmos, ground),
	  groundProvider(nullptr),
	  uniformProvider(std::make_unique<UniformGroundProvider>(ground)),
	  currentGround(ground),
	  rollStepCounter(0)
{
	// For backward compatibility, use the uniform provider
	groundProvider = uniformProvider.get();
}

FlightSimulator::FlightSimulator(
	GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
	const struct atmosphericData &atmos, const GroundProvider &groundProvider)
	: currentPhase(Phase::Aerial), initialized(false),
	  aerialPhase(physicsVars, ball, atmos, GroundSurface{}),
	  bouncePhase(physicsVars, ball, atmos, GroundSurface{}),
	  rollPhase(physicsVars, ball, atmos, GroundSurface{}),
	  groundProvider(&groundProvider),
	  uniformProvider(nullptr),
	  currentGround{},
	  rollStepCounter(0)
{
}

void FlightSimulator::initialize(const BallState &initialState)
{
	state = initialState;
	currentPhase = Phase::Aerial;
	initialized = true;

	// Initialize the aerial phase with the starting state
	aerialPhase.initialize(state);
}

void FlightSimulator::step(float dt)
{
	// Ensure that the simulator has been initialized
	if (!initialized)
	{
		throw std::logic_error("FlightSimulator::step() called before initialize()");
	}

	// Don't step if already complete
	if (currentPhase == Phase::Complete)
	{
		return;
	}

	// Update ground before bounce impact or periodically during roll
	if (currentPhase == Phase::Bounce)
	{
		// Check if ball is about to hit ground (on or near ground, moving downward)
		if (state.position[2] <= currentGround.height + 0.1F && state.velocity[2] < 0.0F)
		{
			updateGroundAtPosition(state.position[0], state.position[1]);
		}
	}
	else if (currentPhase == Phase::Roll)
	{
		// Update ground every 10 steps during roll (every 0.1s at dt=0.01)
		if (++rollStepCounter >= 10)
		{
			updateGroundAtPosition(state.position[0], state.position[1]);
			rollStepCounter = 0;
		}
	}

	// Execute physics step for current phase
	switch (currentPhase)
	{
	case Phase::Aerial:
		aerialPhase.calculateStep(state, dt);
		break;
	case Phase::Bounce:
		bouncePhase.calculateStep(state, dt);
		break;
	case Phase::Roll:
		rollPhase.calculateStep(state, dt);
		break;
	case Phase::Complete:
		// Already complete, do nothing
		break;
	}

	// Check if we need to transition to the next phase
	checkPhaseTransition();
}

bool FlightSimulator::isComplete() const
{
	return currentPhase == Phase::Complete;
}

const BallState &FlightSimulator::getState() const
{
	// Ensure that the simulator has been initialized
	if (!initialized)
	{
		throw std::logic_error("FlightSimulator::getState() called before initialize()");
	}
	return state;
}

const char *FlightSimulator::getCurrentPhaseName() const
{
	switch (currentPhase)
	{
	case Phase::Aerial:
		return "aerial";
	case Phase::Bounce:
		return "bounce";
	case Phase::Roll:
		return "roll";
	case Phase::Complete:
		return "complete";
	default:
		return "unknown";
	}
}

void FlightSimulator::checkPhaseTransition()
{
	switch (currentPhase)
	{
	case Phase::Aerial:
		// Update ground at current position before checking if aerial phase is complete
		updateGroundAtPosition(state.position[0], state.position[1]);
		if (aerialPhase.isPhaseComplete(state))
		{
			currentPhase = Phase::Bounce;
		}
		break;

	case Phase::Bounce:
		if (bouncePhase.isPhaseComplete(state))
		{
			// Update ground before transitioning to roll
			updateGroundAtPosition(state.position[0], state.position[1]);
			currentPhase = Phase::Roll;
			rollStepCounter = 0;  // Reset counter for roll phase
		}
		break;

	case Phase::Roll:
		if (rollPhase.isPhaseComplete(state))
		{
			currentPhase = Phase::Complete;
		}
		break;

	case Phase::Complete:
		// Already complete, stay in this state
		break;
	}
}

void FlightSimulator::updateGroundAtPosition(float x, float y)
{
	if (groundProvider != nullptr)
	{
		currentGround = groundProvider->getGroundAt(x, y);

		// Update all phase objects with new ground properties
		aerialPhase.updateGround(currentGround);
		bouncePhase.updateGround(currentGround);
		rollPhase.updateGround(currentGround);
	}
}
