#include "FlightSimulator.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "physics_constants.hpp"
#include "ground_surface.hpp"

#include <stdio.h>
#include <cmath>
#include <vector>

int main()
{
    const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
    const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

    GroundSurface ground; // Default ground properties

    GolfBallPhysicsVariables physVars(ball, atmos);
    FlightSimulator sim(physVars, ball, atmos, ground);

    // Setup initial state from sensor data
    const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
    Vector3D start_pos{
        ball.x0 * physics_constants::YARDS_TO_FEET,
        ball.y0 * physics_constants::YARDS_TO_FEET,
        ball.z0 * physics_constants::YARDS_TO_FEET
    };

    BallState initialState = BallState::fromLaunchParameters(
        v0_fps,
        ball.launchAngle,
        ball.direction,
        start_pos,
        physics_constants::GRAVITY_FT_PER_S2,
        physVars.getROmega()  // Initial spin from backspin/sidespin
    );

    sim.initialize(initialState);

    // Collect trajectory points
    std::vector<Vector3D> trajectory;
    const float dt = 0.01F;

    while (!sim.isComplete())
    {
        const BallState& state = sim.getState();
        trajectory.push_back(state.position);
        sim.step(dt);
    }

    // Add final position
    trajectory.push_back(sim.getState().position);

    printf("Entire ball trajectory:\n");

    for (const auto& entry : trajectory)
    {
        printf("%.1f %.1f %.1f\n",
               entry[0] / physics_constants::YARDS_TO_FEET,
               entry[1] / physics_constants::YARDS_TO_FEET,
               entry[2]);
    }

    return 0;
}