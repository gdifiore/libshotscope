/**
 * @file test_flight_simulator.cpp
 * @brief Unit tests for the FlightSimulator class
 */

#include <gtest/gtest.h>
#include <cmath>
#include "FlightSimulator.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "physics_constants.hpp"

class FlightSimulatorTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		// Standard test ball parameters
		ball = {
			0.0F,    // x0
			0.0F,    // y0
			0.0F,    // z0
			100.0F,  // exitSpeed (mph)
			30.0F,   // launchAngle (degrees)
			0.0F,    // direction (degrees)
			3000.0F, // backspin (rpm)
			0.0F     // sidespin (rpm)
		};

		// Standard atmospheric conditions
		atmos = {
			70.0F,  // temperature (F)
			0.0F,   // wind x
			0.0F,   // wind y
			0.0F,   // wind z
			0.0F,   // wind direction
			50.0F,  // humidity
			29.92F  // pressure
		};

		// Standard ground conditions
		ground.height = 0.0F;
		ground.restitution = 0.4F;
		ground.frictionStatic = 0.5F;
		ground.frictionDynamic = 0.2F;
		ground.firmness = 0.8F;
	}

	golfBall ball;
	atmosphericData atmos;
	GroundSurface ground;
};

TEST_F(FlightSimulatorTest, InitializesCorrectly)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Setup initial state
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	BallState state = BallState::fromLaunchParameters(v0_fps, ball.launchAngle, ball.direction);

	sim.initialize(state);

	EXPECT_FALSE(sim.isComplete());
	EXPECT_STREQ(sim.getCurrentPhaseName(), "aerial");
}

TEST_F(FlightSimulatorTest, AutomaticallyTransitionsToComplete)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Setup initial state
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	BallState state = BallState::fromLaunchParameters(v0_fps, ball.launchAngle, ball.direction);

	sim.initialize(state);

	// Run simulation to completion
	const float dt = 0.01F;
	const int maxSteps = 2000;
	int steps = 0;

	while (!sim.isComplete() && steps < maxSteps)
	{
		sim.step(dt);
		steps++;
	}

	EXPECT_TRUE(sim.isComplete());
	EXPECT_STREQ(sim.getCurrentPhaseName(), "complete");
	EXPECT_LT(steps, maxSteps) << "Simulation should complete before max steps";
}

TEST_F(FlightSimulatorTest, TransitionsThroughAllPhases)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Setup initial state
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	BallState state = BallState::fromLaunchParameters(v0_fps, ball.launchAngle, ball.direction);

	sim.initialize(state);

	// Track phase transitions
	bool seenAerial = false;
	bool seenBounce = false;
	bool seenRoll = false;
	bool seenComplete = false;

	const float dt = 0.01F;
	const int maxSteps = 2000;

	for (int i = 0; i < maxSteps && !sim.isComplete(); ++i)
	{
		const char* phase = sim.getCurrentPhaseName();

		if (strcmp(phase, "aerial") == 0) seenAerial = true;
		if (strcmp(phase, "bounce") == 0) seenBounce = true;
		if (strcmp(phase, "roll") == 0) seenRoll = true;

		sim.step(dt);
	}

	// Check if we reached complete phase after loop
	if (strcmp(sim.getCurrentPhaseName(), "complete") == 0) seenComplete = true;

	// Should have seen all phases in sequence
	EXPECT_TRUE(seenAerial) << "Should transition through aerial phase";
	EXPECT_TRUE(seenBounce) << "Should transition through bounce phase";
	EXPECT_TRUE(seenRoll) << "Should transition through roll phase";
	EXPECT_TRUE(seenComplete) << "Should eventually reach complete phase";
}

TEST_F(FlightSimulatorTest, ProducesReasonableTrajectory)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Setup initial state
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	BallState state = BallState::fromLaunchParameters(v0_fps, ball.launchAngle, ball.direction);

	sim.initialize(state);

	// Track max height and final distance
	float maxHeight = 0.0F;
	const float dt = 0.01F;
	const int maxSteps = 2000;

	for (int i = 0; i < maxSteps && !sim.isComplete(); ++i)
	{
		const BallState& currentState = sim.getState();
		maxHeight = std::max(maxHeight, currentState.position[2]);
		sim.step(dt);
	}

	const BallState& finalState = sim.getState();
	float finalDistance = std::sqrt(
		finalState.position[0] * finalState.position[0] +
		finalState.position[1] * finalState.position[1]
	);

	// Sanity checks for 100 mph, 30 degree shot
	EXPECT_GT(maxHeight, 10.0F) << "Ball should reach reasonable max height";
	EXPECT_LT(maxHeight, 120.0F) << "Max height should be realistic";
	EXPECT_GT(finalDistance, 100.0F) << "Ball should travel reasonable distance";
	EXPECT_LT(finalDistance, 1000.0F) << "Distance should be realistic";
	EXPECT_NEAR(finalState.position[2], ground.height, 0.1F) << "Ball should end on ground";
}

TEST_F(FlightSimulatorTest, StepsDoNothingWhenComplete)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Setup initial state
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	BallState state = BallState::fromLaunchParameters(v0_fps, ball.launchAngle, ball.direction);

	sim.initialize(state);

	// Run to completion
	const float dt = 0.01F;
	while (!sim.isComplete())
	{
		sim.step(dt);
	}

	// Save final state
	BallState finalState = sim.getState();

	// Additional steps should not change anything
	for (int i = 0; i < 10; ++i)
	{
		sim.step(dt);
	}

	const BallState& afterState = sim.getState();
	EXPECT_EQ(finalState.position[0], afterState.position[0]);
	EXPECT_EQ(finalState.position[1], afterState.position[1]);
	EXPECT_EQ(finalState.position[2], afterState.position[2]);
	EXPECT_EQ(finalState.currentTime, afterState.currentTime);
}

TEST_F(FlightSimulatorTest, HandlesDifferentGroundConditions)
{
	// Test with very bouncy ground
	GroundSurface bouncyGround;
	bouncyGround.height = 0.0F;
	bouncyGround.restitution = 0.8F;  // Very bouncy
	bouncyGround.frictionStatic = 0.2F;
	bouncyGround.frictionDynamic = 0.1F;
	bouncyGround.firmness = 1.0F;

	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, bouncyGround);

	BallState state;
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	const float theta_rad = ball.launchAngle * M_PI / 180.0F;

	state.position = {0.0F, 0.0F, 0.0F};
	state.velocity = {v0_fps * std::cos(theta_rad), 0.0F, v0_fps * std::sin(theta_rad)};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	sim.initialize(state);

	// Run to completion
	const float dt = 0.01F;
	while (!sim.isComplete())
	{
		sim.step(dt);
	}

	// Should still complete successfully
	EXPECT_TRUE(sim.isComplete());
}

TEST_F(FlightSimulatorTest, HandlesNonZeroGroundHeight)
{
	// Test with ground elevated above z=0
	GroundSurface elevatedGround;
	elevatedGround.height = 10.0F;  // Ground at 10 feet elevation
	elevatedGround.restitution = 0.4F;
	elevatedGround.frictionStatic = 0.5F;
	elevatedGround.frictionDynamic = 0.2F;
	elevatedGround.firmness = 0.8F;

	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, elevatedGround);

	// Start ball at ground level (z = 10.0)
	BallState state;
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	const float theta_rad = ball.launchAngle * M_PI / 180.0F;

	state.position = {0.0F, 0.0F, elevatedGround.height};
	state.velocity = {v0_fps * std::cos(theta_rad), 0.0F, v0_fps * std::sin(theta_rad)};
	state.acceleration = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};
	state.currentTime = 0.0F;

	sim.initialize(state);

	// Run to completion
	const float dt = 0.01F;
	const int maxSteps = 2000;
	int steps = 0;

	while (!sim.isComplete() && steps < maxSteps)
	{
		sim.step(dt);
		steps++;

		const BallState& currentState = sim.getState();

		// After initial bounce, ball should not go significantly below ground
		// Allow some tolerance during aerial-to-bounce transition
		if (steps > 50)  // After initial aerial phase
		{
			EXPECT_GE(currentState.position[2], elevatedGround.height - 1.0F)
				<< "Ball should not go far below ground level at step " << steps;
		}
	}

	// Should complete successfully
	EXPECT_TRUE(sim.isComplete()) << "Simulation should complete";
	EXPECT_LT(steps, maxSteps) << "Simulation should complete before max steps";

	// Final position should be at ground height
	const BallState& finalState = sim.getState();
	EXPECT_NEAR(finalState.position[2], elevatedGround.height, 0.1F)
		<< "Ball should end at ground height";

	// Final velocity should be near zero
	float finalSpeed = std::sqrt(
		finalState.velocity[0] * finalState.velocity[0] +
		finalState.velocity[1] * finalState.velocity[1] +
		finalState.velocity[2] * finalState.velocity[2]
	);
	EXPECT_LT(finalSpeed, 1.0F) << "Ball should be stopped or nearly stopped";
}

TEST_F(FlightSimulatorTest, SpinDecaysAcrossAllPhases)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Setup initial state with spin
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	const float initialSpinRadS = physicsVars.getROmega(); // Get initial spin in rad/s
	BallState state = BallState::fromLaunchParameters(
		v0_fps, ball.launchAngle, ball.direction,
		Vector3D{0.0F, 0.0F, 0.0F},
		physics_constants::GRAVITY_FT_PER_S2,
		initialSpinRadS
	);

	sim.initialize(state);

	// Track spin values across phases
	float spinAtStartOfAerial = 0.0F;
	float spinAtEndOfAerial = 0.0F;
	float spinBeforeBounce = 0.0F;
	float spinAfterBounce = 0.0F;
	float spinAtStartOfRoll = 0.0F;
	float spinAtEndOfRoll = 0.0F;

	bool inAerial = true;
	bool inBounce = false;
	bool capturedAerialStart = false;
	bool capturedBounceImpact = false;

	const float dt = 0.01F;
	const int maxSteps = 2000;

	for (int i = 0; i < maxSteps && !sim.isComplete(); ++i)
	{
		const char* phase = sim.getCurrentPhaseName();
		const BallState& currentState = sim.getState();

		// Capture spin at start of aerial phase
		if (strcmp(phase, "aerial") == 0 && !capturedAerialStart)
		{
			spinAtStartOfAerial = currentState.spinRate;
			capturedAerialStart = true;
		}

		// Detect transition from aerial to bounce
		if (strcmp(phase, "bounce") == 0 && inAerial)
		{
			inAerial = false;
			inBounce = true;
			spinAtEndOfAerial = currentState.spinRate;
		}

		// Capture spin before and after bounce impact
		if (strcmp(phase, "bounce") == 0 && !capturedBounceImpact)
		{
			if (currentState.position[2] <= ground.height + 0.01F)
			{
				spinBeforeBounce = currentState.spinRate;
				sim.step(dt);
				spinAfterBounce = sim.getState().spinRate;
				capturedBounceImpact = true;
				continue;
			}
		}

		// Detect transition from bounce to roll
		if (strcmp(phase, "roll") == 0 && inBounce)
		{
			inBounce = false;
			spinAtStartOfRoll = currentState.spinRate;
		}

		sim.step(dt);
	}

	spinAtEndOfRoll = sim.getState().spinRate;

	// Verify spin behavior
	EXPECT_GT(spinAtStartOfAerial, 0.0F) << "Should start with non-zero spin";

	// Spin should decay during aerial phase
	EXPECT_LT(spinAtEndOfAerial, spinAtStartOfAerial)
		<< "Spin should decay during aerial phase";

	// Spin should be reduced on bounce impact (if we captured it)
	if (capturedBounceImpact)
	{
		EXPECT_LT(spinAfterBounce, spinBeforeBounce)
			<< "Spin should be reduced on bounce impact";
		EXPECT_NEAR(spinAfterBounce, spinBeforeBounce * ground.spinRetention, 0.1F)
			<< "Spin reduction should match ground.spinRetention";
	}

	// Spin should continue to decay during roll phase
	if (spinAtStartOfRoll > 0.0F)
	{
		EXPECT_LE(spinAtEndOfRoll, spinAtStartOfRoll)
			<< "Spin should decay during roll phase";
	}

	// Final spin should be less than or equal to initial spin
	EXPECT_LE(sim.getState().spinRate, initialSpinRadS)
		<< "Final spin should be less than or equal to initial spin";
}

TEST_F(FlightSimulatorTest, ThrowsExceptionWhenStepCalledBeforeInitialize)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Should throw exception when stepping before initialization
	EXPECT_THROW({
		sim.step(0.01F);
	}, std::logic_error);
}

TEST_F(FlightSimulatorTest, ThrowsExceptionWhenGetStateCalledBeforeInitialize)
{
	GolfBallPhysicsVariables physicsVars(ball, atmos);
	FlightSimulator sim(physicsVars, ball, atmos, ground);

	// Should throw exception when getting state before initialization
	EXPECT_THROW({
		[[maybe_unused]] const BallState& state = sim.getState();
	}, std::logic_error);
}
