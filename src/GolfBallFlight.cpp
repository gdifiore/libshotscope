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
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "physics_constants.hpp"

#include <cmath>

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
 * Initializes the golf ball flight by setting the initial position, velocity, and other variables.
 *
 * Some of these have initial values from other classes, some need to be calculated for the first time.
 */
void GolfBallFlight::initialize()
{
	position = {ball.x0, ball.y0, ball.z0};

	velocity3D = physicsVars.getV0Vector();
	v = sqrt(velocity3D[0] * velocity3D[0] + velocity3D[1] * velocity3D[1] +
			 velocity3D[2] * velocity3D[2]);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;
	calculateVelocityw();
	vw = v;

	calculatePhi();
	calculateTau();
	calculateRw();
	calculateRe_x_e5();
	calculateSpinFactor();

	calculateAccelD();
	calculateAccelM();
	calculateAccel();
}

void GolfBallFlight::calculatePosition()
{
	position[0] =
		position[0] + velocity3D[0] * physics_constants::SIMULATION_TIME_STEP + physics_constants::HALF * acceleration3D[0] * physics_constants::SIMULATION_TIME_STEP * physics_constants::SIMULATION_TIME_STEP;
	position[1] =
		position[1] + velocity3D[1] * physics_constants::SIMULATION_TIME_STEP + physics_constants::HALF * acceleration3D[1] * physics_constants::SIMULATION_TIME_STEP * physics_constants::SIMULATION_TIME_STEP;
	position[2] =
		position[2] + velocity3D[2] * physics_constants::SIMULATION_TIME_STEP + physics_constants::HALF * acceleration3D[2] * physics_constants::SIMULATION_TIME_STEP * physics_constants::SIMULATION_TIME_STEP;
}

void GolfBallFlight::calculateV()
{
	float vx = velocity3D[0] + acceleration3D[0] * physics_constants::SIMULATION_TIME_STEP;
	float vy = velocity3D[1] + acceleration3D[1] * physics_constants::SIMULATION_TIME_STEP;
	float vz = velocity3D[2] + acceleration3D[2] * physics_constants::SIMULATION_TIME_STEP;

	velocity3D = {vx, vy, vz};

	v = sqrt(vx * vx + vy * vy + vz * vz);
	vMph = v / physics_constants::MPH_TO_FT_PER_S;
}

// In physics, "velocity_w" typically refers to the velocity of an object in the horizontal direction.
// It represents the speed at which the object is moving horizontally, without considering any vertical motion.
void GolfBallFlight::calculateVelocityw()
{
	if (position[2] >= atmos.hWind)
	{
		vw = sqrt(pow(velocity3D[0] - velocity3D_w[0], 2) +
				  pow(velocity3D[1] - velocity3D_w[1], 2) + pow(velocity3D[2], 2));
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

void GolfBallFlight::calculateAccel()
{
	acceleration3D[0] = accelerationDrag3D[0] + accelertaionMagnitude3D[0];
	acceleration3D[1] = accelerationDrag3D[1] + accelertaionMagnitude3D[1];
	acceleration3D[2] =
		accelerationDrag3D[2] + accelertaionMagnitude3D[2] - physics_constants::GRAVITY_FT_PER_S2;
}

void GolfBallFlight::calculateAccelD()
{
	accelerationDrag3D[0] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * (velocity3D[0] - velocity3D_w[0]);
	accelerationDrag3D[1] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * (velocity3D[1] - velocity3D_w[1]);
	accelerationDrag3D[2] = -physicsVars.getC0() * determineCoefficientOfDrag() *
							vw * velocity3D[2];
}

void GolfBallFlight::calculateAccelM()
{
	accelertaionMagnitude3D[0] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[1] * velocity3D[2] -
		 physicsVars.getW()[2] * (velocity3D[1] - velocity3D_w[1])) /
		w_perp_div_w;
	accelertaionMagnitude3D[1] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[2] * (velocity3D[0] - velocity3D_w[0]) -
		 physicsVars.getW()[0] * velocity3D[2]) /
		w_perp_div_w;
	accelertaionMagnitude3D[2] =
		physicsVars.getC0() *
		(determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
		(physicsVars.getW()[0] * (velocity3D[1] - velocity3D_w[1]) -
		 physicsVars.getW()[1] * (velocity3D[0] - velocity3D_w[0])) /
		w_perp_div_w;
}

void GolfBallFlight::calculatePhi()
{
	phi = atan2(position[1], position[2]) * 180 / M_PI;
}

void GolfBallFlight::calculateTau()
{
	tau =
		1 / (physics_constants::TAU_COEFF * v / (physics_constants::STD_BALL_CIRCUMFERENCE_IN / (2 * M_PI * physics_constants::INCHES_PER_FOOT)));
}

void GolfBallFlight::calculateRw()
{
	rw = physicsVars.getROmega() * exp(-currentTime / tau);
}

void GolfBallFlight::calculateRe_x_e5()
{
	Re_x_e5 = (vwMph / physics_constants::RE_VELOCITY_DIVISOR) * physicsVars.getRe100() * physics_constants::RE_SCALE_FACTOR;
}

float GolfBallFlight::determineCoefficientOfDrag()
{
	if (getRe_x_e5() <= physics_constants::RE_THRESHOLD_LOW)
	{
		return physics_constants::CD_LOW;
	}
	else if (getRe_x_e5() < physics_constants::RE_THRESHOLD_HIGH)
	{
		return physics_constants::CD_LOW -
			   (physics_constants::CD_LOW - physics_constants::CD_HIGH) * (Re_x_e5 - physics_constants::RE_THRESHOLD_LOW) /
				   physics_constants::RE_THRESHOLD_LOW +
			   physics_constants::CD_SPIN * spinFactor;
	}
	else
	{
		return physics_constants::CD_HIGH + physics_constants::CD_SPIN * spinFactor;
	}
}

float GolfBallFlight::determineCoefficientOfLift()
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

void GolfBallFlight::calculateSpinFactor()
{
	spinFactor = rw / vw;
}

/**
 * Calculates a single step in the golf ball's flight.
 * Updates the current time and calculates various parameters such as position, velocity, spin, and acceleration.
 */
void GolfBallFlight::calculateFlightStep()
{
	currentTime += physics_constants::SIMULATION_TIME_STEP;

	calculatePosition();
	calculateV();

	calculateVelocityw();

	calculatePhi();
	calculateTau();
	calculateRw();

	calculateRe_x_e5();
	calculateSpinFactor();

	calculateAccelD();
	calculateAccelM();
	calculateAccel();
}