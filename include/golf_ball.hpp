#ifndef GOLF_BALL_HPP
#define GOLF_BALL_HPP

/**
 * @brief Golf ball launch parameters
 *
 * Coordinate system:
 *   x-axis: Lateral direction (positive = right of target line)
 *   y-axis: Forward/downrange direction (direction=0 points along +y)
 *   z-axis: Vertical/height (positive = up)
 */
struct golfBall
{
	/**
	 * @brief Initial lateral position (in yards).
	 *
	 * Positive values are right of target line.
	 */
	float x0;

	/**
	 * @brief Initial forward position (in yards).
	 *
	 * Distance downrange from origin.
	 */
	float y0;

	/**
	 * @brief Initial height above ground (in yards).
	 *
	 * Typically 0.0 for ground-level shots.
	 */
	float z0;

	/**
	 * @brief Ball speed at impact (in mph).
	 *
	 * Typical values:
	 * - Driver: ~150-180 mph
	 * - Mid iron: ~100-120 mph
	 * - Wedge: ~70-90 mph
	 */
	float exitSpeed;

	/**
	 * @brief Vertical launch angle (in degrees).
	 *
	 * Angle above horizontal plane.
	 * Typical values:
	 * - Driver: ~10-15 deg
	 * - Mid iron: ~20-30 deg
	 * - Wedge: ~40-50 deg
	 */
	float launchAngle;

	/**
	 * @brief Horizontal launch direction (in degrees).
	 *
	 * Angle from target line (0 = straight, + = right, - = left).
	 * Typical range: -45 to +45 deg
	 */
	float direction;

	/**
	 * @brief Backspin rate (in rpm).
	 *
	 * Positive values indicate backspin.
	 * Typical values:
	 * - Driver: ~2000-3000 rpm
	 * - Mid iron: ~5000-7000 rpm
	 * - Wedge: ~8000-10000 rpm
	 */
	float backspin;

	/**
	 * @brief Sidespin rate (in rpm).
	 *
	 * Positive values produce hook spin, negative values produce slice spin.
	 * Typical range: -3000 to +3000 rpm
	 */
	float sidespin;
};

/**
 * @brief Golf ball landing information
 */
struct golfBallLanding
{
	/**
	 * @brief Final lateral position (in yards).
	 *
	 * Positive values are right of target line.
	 */
	float xF;

	/**
	 * @brief Final forward position (in yards).
	 *
	 * Distance downrange from origin at rest.
	 */
	float yF;

	/**
	 * @brief Final height above ground (in yards).
	 *
	 * Typically 0.0 when ball comes to rest on ground.
	 */
	float zF;

	/**
	 * @brief Total time of flight (in seconds).
	 *
	 * Duration from launch to final rest position.
	 */
	float timeOfFlight;

	/**
	 * @brief Direction from start to landing (in degrees).
	 *
	 * Bearing angle from initial position to final position.
	 */
	float bearing;

	/**
	 * @brief Total distance from start to landing (in yards).
	 *
	 * Straight-line distance between initial and final positions.
	 */
	float distance;
};

#endif // GOLF_BALL_HPP