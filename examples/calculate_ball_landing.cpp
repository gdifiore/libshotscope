#include "FlightSimulator.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "physics_constants.hpp"
#include "ground_surface.hpp"

#include <stdio.h>
#include <cmath>

int main()
{
    const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
    const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

    GroundSurface ground; // Default ground properties

    GolfBallPhysicsVariables physVars(ball, atmos);
    FlightSimulator sim(physVars, ball, atmos, ground);

    // Setup initial state - uses sensible defaults for position, acceleration, and time
    const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
    BallState initialState = BallState::fromLaunchParameters(
        v0_fps,
        ball.launchAngle,
        ball.direction
    );

    sim.initialize(initialState);

    // Run simulation to completion
    const float dt = 0.01F;
    while (!sim.isComplete())
    {
        sim.step(dt);
    }

    const BallState& finalState = sim.getState();
    printf("Landing spot: %.1f %.1f %.1f yards\n",
           finalState.position[0] / physics_constants::YARDS_TO_FEET,
           finalState.position[1] / physics_constants::YARDS_TO_FEET,
           finalState.position[2]);

    return 0;
}