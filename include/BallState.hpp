#ifndef BALLSTATE_HPP
#define BALLSTATE_HPP

#include "math_utils.hpp"
#include "physics_constants.hpp"
#include <cmath>

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

	/**
	 * @brief Creates a BallState from launch parameters with sensible defaults.
	 *
	 * This factory method calculates the initial velocity vector from launch speed,
	 * angle, and direction, while providing sensible defaults for position (origin),
	 * acceleration (earth gravity), and time (0).
	 *
	 * @param speed_fps Exit speed in feet per second
	 * @param launch_angle_deg Launch angle in degrees
	 * @param direction_deg Direction angle in degrees (0 = straight, positive = right)
	 * @param start_pos Starting position (default: origin)
	 * @param gravity Gravitational acceleration in ft/s² (default: earth gravity)
	 * @return BallState initialized with calculated velocity and default values
	 */
	static BallState fromLaunchParameters(
		float speed_fps,
		float launch_angle_deg,
		float direction_deg,
		const Vector3D &start_pos = Vector3D{0.0F, 0.0F, 0.0F},
		float gravity = physics_constants::GRAVITY_FT_PER_S2)
	{
		const float theta_rad = launch_angle_deg * M_PI / 180.0F;
		const float phi_rad = direction_deg * M_PI / 180.0F;

		Vector3D velocity{
			speed_fps * std::cos(theta_rad) * std::cos(phi_rad),
			speed_fps * std::cos(theta_rad) * std::sin(phi_rad),
			speed_fps * std::sin(theta_rad)
		};

		Vector3D acceleration{0.0F, 0.0F, -gravity};

		return BallState(start_pos, velocity, acceleration, 0.0F);
	}

	// State variables
	Vector3D position;      // Current position (x, y, z) in feet
	Vector3D velocity;      // Current velocity (vx, vy, vz) in ft/s
	Vector3D acceleration;  // Current acceleration (ax, ay, az) in ft/s²
	float currentTime;      // Current simulation time in seconds
};

#endif // BALLSTATE_HPP
