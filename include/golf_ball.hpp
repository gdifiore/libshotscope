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
    float x0;          // yards; initial lateral position (right of target line)
    float y0;          // yards; initial forward position (downrange)
    float z0;          // yards; initial height above ground
    float exitSpeed;   // mph; ball speed at impact
    float launchAngle; // deg; vertical angle above horizontal (0=flat, 90=straight up)
    float direction;   // deg; horizontal angle (0=straight, >0=right, <0=left)
    float backspin;    // rpm; backspin rate (positive = backspin)
    float sidespin;    // rpm; sidespin rate (+ for hook, - for slice)
};

/**
 * @brief Golf ball landing information
 */
struct golfBallLanding
{
    float xF;           // yards; final lateral position (right of target line)
    float yF;           // yards; final forward position (downrange)
    float zF;           // yards; final height above ground
    float timeOfFlight; // seconds; total time in air
    float bearing;      // degrees; direction from start to landing
    float distance;     // yards; total distance from start to landing
};

#endif // GOLF_BALL_HPP