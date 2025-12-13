/**
 * @file FlightPhase.cpp
 * @author Gabriel DiFiore
 * @brief Contains the implementation of flight phase classes.
 *
 * This file defines the different flight phases for golf ball simulation:
 * AerialPhase, BouncePhase, and RollPhase.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
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
	const struct atmosphericData &atmos, const GroundSurface &ground)
	: physicsVars(physicsVars), ball(ball), atmos(atmos), ground(ground)
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
	accelerationMagnitude3D = {0.0F, 0.0F, 0.0F};
}

void AerialPhase::initialize(BallState &state)
{
	// Initialize spin from physicsVars if not already set
	if (state.spinRate == 0.0F)
	{
		state.spinRate = physicsVars.getROmega();
	}

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

void AerialPhase::calculateAccelerations(BallState &state)
{
	// Calculate derived values and accelerations without modifying position or time
	v = sqrt(state.velocity[0] * state.velocity[0] + state.velocity[1] * state.velocity[1] +
			 state.velocity[2] * state.velocity[2]);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;

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

void AerialPhase::calculateStep(BallState &state, float dt)
{
	state.currentTime += dt;

	// Update spin with exponential decay
	calculateTau();
	state.spinRate = state.spinRate * exp(-dt / tau);

	calculatePosition(state, dt);
	calculateV(state, dt);
	calculateVelocityw(state);
	calculatePhi(state);
	calculateRw(state);
	calculateRe_x_e5();
	calculateSpinFactor();
	calculateAccelD(state);
	calculateAccelM(state);
	calculateAccel(state);
}

bool AerialPhase::isPhaseComplete(const BallState &state) const
{
	// Aerial phase is complete when ball reaches ground level
	// Use small epsilon for robust float comparison
	const float GROUND_CONTACT_EPSILON = 0.01F;
	return state.position[2] < (ground.height - GROUND_CONTACT_EPSILON);
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

	vwMph = vw / physics_constants::MPH_TO_FT_PER_S;
}

void AerialPhase::calculatePhi(BallState &state)
{
	phi = atan2(state.position[1], state.position[2]) * 180 / physics_constants::PI;
}

void AerialPhase::calculateTau()
{
	tau = 1 / (physics_constants::TAU_COEFF * v /
		(physics_constants::STD_BALL_CIRCUMFERENCE_IN / (2 * physics_constants::PI * physics_constants::INCHES_PER_FOOT)));
}

void AerialPhase::calculateRw(BallState &state)
{
	// Use current spin rate from state (which decays over time)
	rw = state.spinRate;
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
	if (vw < physics_constants::MIN_VELOCITY_THRESHOLD)
	{
		spinFactor = 0.0F; // Ball essentially stationary, no meaningful spin factor
	}
	else
	{
		spinFactor = rw / vw;
	}
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
	accelerationMagnitude3D[0] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[1] * state.velocity[2] -
		 physicsVars.getW()[2] * (state.velocity[1] - velocity3D_w[1])) /
		w_perp_div_w;
	accelerationMagnitude3D[1] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[2] * (state.velocity[0] - velocity3D_w[0]) -
		 physicsVars.getW()[0] * state.velocity[2]) /
		w_perp_div_w;
	accelerationMagnitude3D[2] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[0] * (state.velocity[1] - velocity3D_w[1]) -
		 physicsVars.getW()[1] * (state.velocity[0] - velocity3D_w[0])) /
		w_perp_div_w;
}

void AerialPhase::calculateAccel(BallState &state)
{
	state.acceleration[0] = accelerationDrag3D[0] + accelerationMagnitude3D[0];
	state.acceleration[1] = accelerationDrag3D[1] + accelerationMagnitude3D[1];
	state.acceleration[2] = accelerationDrag3D[2] + accelerationMagnitude3D[2] -
							physics_constants::GRAVITY_FT_PER_S2;
}

void AerialPhase::updateGround(const GroundSurface &newGround)
{
	ground = newGround;
}

// ============================================================================
// BouncePhase Implementation
// ============================================================================

BouncePhase::BouncePhase(
	GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
	const struct atmosphericData &atmos, const GroundSurface &ground)
	: physicsVars(physicsVars), ball(ball), atmos(atmos), ground(ground),
	  aerialPhase(physicsVars, ball, atmos, ground)
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

		if (vHorizontal > physics_constants::MIN_HORIZONTAL_VELOCITY)
		{
			float frictionFactor = 1.0F - ground.frictionStatic * (1.0F - ground.firmness);
			frictionFactor = std::max(0.0F, std::min(1.0F, frictionFactor));

			state.velocity[0] *= frictionFactor;
			state.velocity[1] *= frictionFactor;
		}

		// Reduce spin on impact
		state.spinRate *= ground.spinRetention;
	}

	// Calculate aerodynamic forces (drag, lift, Magnus effect)
	aerialPhase.calculateAccelerations(state);

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
	// Transition to roll only when ball is on ground with low vertical velocity
	if (state.position[2] <= ground.height + physics_constants::GROUND_CONTACT_THRESHOLD &&
		std::abs(state.velocity[2]) < physics_constants::MIN_BOUNCE_VELOCITY)
	{
		return true;
	}

	return false;
}

void BouncePhase::updateGround(const GroundSurface &newGround)
{
	ground = newGround;
	// Also update the embedded aerial phase
	aerialPhase.updateGround(newGround);
}

// ============================================================================
// RollPhase Implementation
// ============================================================================

RollPhase::RollPhase(
	GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
	const struct atmosphericData &atmos, const GroundSurface &ground)
	: physicsVars(physicsVars), ball(ball), atmos(atmos), ground(ground)
{
}

void RollPhase::calculateStep(BallState &state, float dt)
{
	// Calculate horizontal velocity magnitude
	float vHorizontal = sqrt(state.velocity[0] * state.velocity[0] +
							 state.velocity[1] * state.velocity[1]);

	if (vHorizontal > physics_constants::MIN_HORIZONTAL_VELOCITY)
	{
		// Calculate rolling friction deceleration using surface's dynamic friction
		float deceleration = ground.frictionDynamic * physics_constants::GRAVITY_FT_PER_S2;

		// Calculate velocity reduction
		float velocityReduction = deceleration * dt;

		// Don't let velocity reverse direction
		if (velocityReduction >= vHorizontal)
		{
			state.velocity[0] = 0.0F;
			state.velocity[1] = 0.0F;
		}
		else
		{
			// Apply deceleration in direction opposite to motion
			float velocityFactor = (vHorizontal - velocityReduction) / vHorizontal;
			state.velocity[0] *= velocityFactor;
			state.velocity[1] *= velocityFactor;
		}

		// Update position
		state.position[0] += state.velocity[0] * dt;
		state.position[1] += state.velocity[1] * dt;
	}

	// Apply spin decay during rolling (friction with ground)
	// Rolling friction causes faster spin decay than in air
	float spinDecay = physics_constants::ROLL_SPIN_DECAY_RATE * dt;
	if (state.spinRate > spinDecay)
	{
		state.spinRate -= spinDecay;
	}
	else
	{
		state.spinRate = 0.0F;
	}

	// Ball stays on ground during roll
	state.position[2] = ground.height;
	state.velocity[2] = 0.0F;
	state.acceleration[0] = 0.0F;
	state.acceleration[1] = 0.0F;
	state.acceleration[2] = 0.0F;

	state.currentTime += dt;
}

bool RollPhase::isPhaseComplete(const BallState &state) const
{
	// Roll is complete when ball velocity drops below stopping threshold
	float vHorizontal = sqrt(state.velocity[0] * state.velocity[0] +
							 state.velocity[1] * state.velocity[1]);
	return vHorizontal < physics_constants::STOPPING_VELOCITY;
}

void RollPhase::updateGround(const GroundSurface &newGround)
{
	ground = newGround;
}
