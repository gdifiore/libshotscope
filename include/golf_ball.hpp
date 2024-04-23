#ifndef GOLF_BALL_HPP
#define GOLF_BALL_HPP

constexpr float circumference = 5.277; // circumference (in) of a standard golf ball
constexpr float mass = 1.62; // mass (oz) of a standard golf ball

struct golfBall {
    float exitSpeed;
    float launchAngle;
    float direction;
    float backspin;
    float sidespin; // + = hook - = slice
};

struct golfBallLanding {
    float xFinal; // yards
    float yFinal; // yards
    float zFinal; // yards
    float timeOfFlight;
    float bearing;
    float distance; // yards
};

#endif // GOLF_BALL_HPP