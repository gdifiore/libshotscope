#ifndef GOLF_BALL_HPP
#define GOLF_BALL_HPP

struct golfBall
{
    float x0;          // yards; points to the right of the golfer (this needs to be set to dist of ball -> sensor)
    float y0;
    float z0;
    float exitSpeed;   // mph
    float launchAngle; // deg
    float direction;   // deg
    float backspin;    // rpm
    float sidespin;    // rpm (+ for hook - for slice)
};

struct golfBallLanding
{
    float xF; // yards
    float yF;
    float zF;
    float timeOfFlight;
    float bearing;
    float distance; // yards
};

#endif // GOLF_BALL_HPP