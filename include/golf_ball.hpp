#ifndef GOLF_BALL_HPP
#define GOLF_BALL_HPP

struct golfBall
{
    static constexpr float std_golf_ball_circumference = 5.277; // circumference (in) of a standard golf ball
    static constexpr float std_golf_ball_mass = 1.62;           // mass (oz) of a standard golf ball

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