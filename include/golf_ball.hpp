#ifndef GOLF_BALL_HPP
#define GOLF_BALL_HPP

constexpr float circumference = 5.277; // circumference (in) of a standard golf ball
constexpr float mass = 1.62; // mass (oz) of a standard golf ball

struct golfBall {
    float x0; // yards; points to the right of the golfer (this needs to be set to dist of ball -> sensor)
    float y0;
    float z0;
    float exitSpeed;
    float launchAngle;
    float direction;
    float backspin;
    float sidespin; // + = hook - = slice
};

struct golfBallLanding {
    float xF; // yards
    float yF;
    float zF;
    float timeOfFlight;
    float bearing;
    float distance; // yards
};

#endif // GOLF_BALL_HPP