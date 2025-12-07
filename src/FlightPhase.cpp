/**
 * @file FlightPhase.cpp
 * @author Gabriel DiFiore
 * @brief Contains the implementation of flight phase classes.
 *
 * This file defines the different flight phases for golf ball simulation:
 * AerialPhase, BouncePhase, and RollPhase.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#include "FlightPhase.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "physics_constants.hpp"

#include <algorithm>
#include <cmath>

// ============================================================================
// AerialPhase Implementation
// ============================================================================

AerialPhase::AerialPhase(
	GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
	const struct atmosphericData &atmos)
	: physicsVars(physicsVars), ball(ball), atmos(atmos)
{
	// Initialize calculated variables
	v = 0.0F;
	vMph = 0.0F;
	phi = 0.0F;
	tau = 0.0F;
	rw = 0.0F;
	Re_x_e5 = 0.0F;
	vw = 0.0F;
	vwMph = 0.0F;
	spinFactor = 0.0F;
	velocity3D_w = {0.0F, 0.0F, 0.0F};
	accelerationDrag3D = {0.0F, 0.0F, 0.0F};
	accelertaionMagnitude3D = {0.0F, 0.0F, 0.0F};
}

void AerialPhase::initialize(BallState &state)
{
	// Calculate initial derived values without advancing time or position
	v = sqrt(state.velocity[0] * state.velocity[0] + state.velocity[1] * state.velocity[1] +
			 state.velocity[2] * state.velocity[2]);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;

	calculateVelocityw(state);
	vw = v;

	calculatePhi(state);
	calculateTau();
	calculateRw(state);
	calculateRe_x_e5();
	calculateSpinFactor();

	calculateAccelD(state);
	calculateAccelM(state);
	calculateAccel(state);
}

void AerialPhase::calculateStep(BallState &state, float dt)
{
	state.currentTime += dt;

	calculatePosition(state, dt);
	calculateV(state, dt);
	calculateVelocityw(state);
	calculatePhi(state);
	calculateTau();
	calculateRw(state);
	calculateRe_x_e5();
	calculateSpinFactor();
	calculateAccelD(state);
	calculateAccelM(state);
	calculateAccel(state);
}

bool AerialPhase::isPhaseComplete(const BallState &state) const
{
	// Aerial phase is complete when ball reaches ground (z < 0)
	return state.position[2] < 0.0F;
}

void AerialPhase::calculatePosition(BallState &state, float dt)
{
	state.position[0] = state.position[0] + state.velocity[0] * dt +
		physics_constants::HALF * state.acceleration[0] * dt * dt;
	state.position[1] = state.position[1] + state.velocity[1] * dt +
		physics_constants::HALF * state.acceleration[1] * dt * dt;
	state.position[2] = state.position[2] + state.velocity[2] * dt +
		physics_constants::HALF * state.acceleration[2] * dt * dt;
}

void AerialPhase::calculateV(BallState &state, float dt)
{
	float vx = state.velocity[0] + state.acceleration[0] * dt;
	float vy = state.velocity[1] + state.acceleration[1] * dt;
	float vz = state.velocity[2] + state.acceleration[2] * dt;

	state.velocity = {vx, vy, vz};

	v = sqrt(vx * vx + vy * vy + vz * vz);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;
}

void AerialPhase::calculateVelocityw(BallState &state)
{
	if (state.position[2] >= atmos.hWind)
	{
		vw = sqrt(pow(state.velocity[0] - velocity3D_w[0], 2) +
				  pow(state.velocity[1] - velocity3D_w[1], 2) + pow(state.velocity[2], 2));
		velocity3D_w[0] = physicsVars.getVw()[0];
		velocity3D_w[1] = physicsVars.getVw()[1];
	}
	else
	{
		vw = v;
		velocity3D_w[0] = 0;
		velocity3D_w[1] = 0;
	}

	vwMph = v / physics_constants::MPH_TO_FT_PER_S;
}

void AerialPhase::calculatePhi(BallState &state)
{
	phi = atan2(state.position[1], state.position[2]) * 180 / M_PI;
}

void AerialPhase::calculateTau()
{
	tau = 1 / (physics_constants::TAU_COEFF * v /
		(physics_constants::STD_BALL_CIRCUMFERENCE_IN / (2 * M_PI * physics_constants::INCHES_PER_FOOT)));
}

void AerialPhase::calculateRw(BallState &state)
{
	rw = physicsVars.getROmega() * exp(-state.currentTime / tau);
}

void AerialPhase::calculateRe_x_e5()
{
	Re_x_e5 = (vwMph / physics_constants::RE_VELOCITY_DIVISOR) *
		physicsVars.getRe100() * physics_constants::RE_SCALE_FACTOR;
}

float AerialPhase::determineCoefficientOfDrag()
{
	if (getRe_x_e5() <= physics_constants::RE_THRESHOLD_LOW)
	{
		return physics_constants::CD_LOW;
	}
	else if (getRe_x_e5() < physics_constants::RE_THRESHOLD_HIGH)
	{
		return physics_constants::CD_LOW -
			   (physics_constants::CD_LOW - physics_constants::CD_HIGH) *
			   (Re_x_e5 - physics_constants::RE_THRESHOLD_LOW) / physics_constants::RE_THRESHOLD_LOW +
			   physics_constants::CD_SPIN * spinFactor;
	}
	else
	{
		return physics_constants::CD_HIGH + physics_constants::CD_SPIN * spinFactor;
	}
}

float AerialPhase::determineCoefficientOfLift()
{
	if (spinFactor <= physics_constants::SPIN_FACTOR_THRESHOLD)
	{
		return physics_constants::LIFT_COEFF1 * spinFactor +
			   physics_constants::LIFT_COEFF2 * pow(spinFactor, 2);
	}
	else
	{
		return physics_constants::CL_DEFAULT;
	}
}

void AerialPhase::calculateSpinFactor()
{
	spinFactor = rw / vw;
}

void AerialPhase::calculateAccelD(BallState &state)
{
	accelerationDrag3D[0] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * (state.velocity[0] - velocity3D_w[0]);
	accelerationDrag3D[1] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * (state.velocity[1] - velocity3D_w[1]);
	accelerationDrag3D[2] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * state.velocity[2];
}

void AerialPhase::calculateAccelM(BallState &state)
{
	accelertaionMagnitude3D[0] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[1] * state.velocity[2] -
		 physicsVars.getW()[2] * (state.velocity[1] - velocity3D_w[1])) /
		w_perp_div_w;
	accelertaionMagnitude3D[1] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[2] * (state.velocity[0] - velocity3D_w[0]) -
		 physicsVars.getW()[0] * state.velocity[2]) /
		w_perp_div_w;
	accelertaionMagnitude3D[2] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[0] * (state.velocity[1] - velocity3D_w[1]) -
		 physicsVars.getW()[1] * (state.velocity[0] - velocity3D_w[0])) /
		w_perp_div_w;
}

void AerialPhase::calculateAccel(BallState &state)
{
	state.acceleration[0] = accelerationDrag3D[0] + accelertaionMagnitude3D[0];
	state.acceleration[1] = accelerationDrag3D[1] + accelertaionMagnitude3D[1];
	state.acceleration[2] = accelerationDrag3D[2] + accelertaionMagnitude3D[2] -
		physics_constants::GRAVITY_FT_PER_S2;
}

// ============================================================================
// BouncePhase Implementation
// ============================================================================

BouncePhase::BouncePhase(
	GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
	const struct atmosphericData &atmos, const GroundSurface &ground)
	: physicsVars(physicsVars), ball(ball), atmos(atmos), ground(ground),
	  aerialPhase(physicsVars, ball, atmos)
{
}

void BouncePhase::calculateStep(BallState &state, float dt)
{
	// Apply bounce impact when ball contacts ground while moving downward
	if (state.position[2] <= ground.height && state.velocity[2] < 0.0F)
	{
		state.position[2] = ground.height;
		state.velocity[2] = -ground.restitution * state.velocity[2];

		// Apply friction to horizontal velocity components
		float vHorizontal = sqrt(state.velocity[0] * state.velocity[0] +
		                         state.velocity[1] * state.velocity[1]);

		if (vHorizontal > 0.0001F)
		{
			float frictionFactor = 1.0F - ground.frictionStatic * (1.0F - ground.firmness);
			frictionFactor = std::max(0.0F, std::min(1.0F, frictionFactor));

			state.velocity[0] *= frictionFactor;
			state.velocity[1] *= frictionFactor;
		}
	}

	// Calculate aerodynamic forces (drag, lift, Magnus effect)
	aerialPhase.initialize(state);

	// Update position and velocity
	state.position[0] += state.velocity[0] * dt + 0.5F * state.acceleration[0] * dt * dt;
	state.position[1] += state.velocity[1] * dt + 0.5F * state.acceleration[1] * dt * dt;
	state.position[2] += state.velocity[2] * dt + 0.5F * state.acceleration[2] * dt * dt;

	state.velocity[0] += state.acceleration[0] * dt;
	state.velocity[1] += state.acceleration[1] * dt;
	state.velocity[2] += state.acceleration[2] * dt;

	state.currentTime += dt;

	if (state.position[2] < ground.height)
	{
		state.position[2] = ground.height;
	}
}

bool BouncePhase::isPhaseComplete(const BallState &state) const
{
	const float MIN_BOUNCE_VELOCITY = 1.0F;
	const float GROUND_THRESHOLD = 0.1F;

	// Transition to roll only when ball is on ground with low vertical velocity
	if (state.position[2] <= ground.height + GROUND_THRESHOLD &&
	    std::abs(state.velocity[2]) < MIN_BOUNCE_VELOCITY)
	{
		return true;
	}

	return false;
}

// ============================================================================
// RollPhase Implementation (Skeleton)
// ============================================================================

RollPhase::RollPhase(
	GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
	const struct atmosphericData &atmos, const GroundSurface &ground)
	: physicsVars(physicsVars), ball(ball), atmos(atmos), ground(ground)
{
}

void RollPhase::calculateStep(BallState &state, float dt)
{
	// TODO: Implement roll physics
	// This should handle:
	// - Calculate rolling friction force: F = -ground.frictionDynamic * mass * g * velocity_direction
	// - Apply deceleration to horizontal velocity
	// - Calculate spin decay during rolling
	// - Update position: position += velocity * dt
	// - Ensure ball stays on ground: state.position[2] = ground.height
	(void)state; // Suppress unused parameter warning
	(void)dt;
}

bool RollPhase::isPhaseComplete(const BallState &state) const
{
	// TODO: Implement roll completion logic
	// Roll is complete when ball comes to rest (velocity below threshold)
	// Example: return sqrt(state.velocity[0]*state.velocity[0] +
	//                      state.velocity[1]*state.velocity[1]) < 0.01F;
	(void)state; // Suppress unused parameter warning
	return false;
}
