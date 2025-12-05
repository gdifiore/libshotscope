#ifndef BALLSTATE_HPP
#define BALLSTATE_HPP

#include "math_utils.hpp"

/**
 * @brief Represents the physics state of a golf ball at a specific point in time.
 *
 * This class encapsulates the essential state variables needed to track a golf ball's
 * position, velocity, and acceleration during flight, bounce, or roll phases.
 * Separating state from calculations makes it easier to transition between different
 * physics phases while maintaining consistent state data.
 */
class BallState
{
public:
	/**
	 * @brief Constructs a BallState with zero-initialized values.
	 */
	BallState() : position{0.0F, 0.0F, 0.0F},
	              velocity{0.0F, 0.0F, 0.0F},
	              acceleration{0.0F, 0.0F, 0.0F},
	              currentTime(0.0F) {}

	/**
	 * @brief Constructs a BallState with specified initial values.
	 *
	 * @param pos Initial position vector
	 * @param vel Initial velocity vector
	 * @param accel Initial acceleration vector
	 * @param time Initial time value
	 */
	BallState(const Vector3D &pos, const Vector3D &vel, const Vector3D &accel, float time)
		: position(pos), velocity(vel), acceleration(accel), currentTime(time) {}

	// State variables
	Vector3D position;      ///< Current position (x, y, z) in feet
	Vector3D velocity;      ///< Current velocity (vx, vy, vz) in ft/s
	Vector3D acceleration;  ///< Current acceleration (ax, ay, az) in ft/s²
	float currentTime;      ///< Current simulation time in seconds
};

#endif // BALLSTATE_HPP
