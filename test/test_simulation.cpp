#include <gtest/gtest.h>

#include "GolfBallFlight.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "Simulator.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "math_constants.hpp"
#include "math_utils.hpp"

TEST(ShotScopeTest, flightLanding)
{
    const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
    const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

    GolfBallPhysicsVariables physVars(ball, atmos);
    GolfBallFlight flight(physVars, ball, atmos);

    Simulator simulator(physVars, flight);

    Vector3D result = simulator.runSimulationLanding();
    EXPECT_NEAR(result[0], 0.0, 0.1);
    EXPECT_NEAR(result[1], 269.0, 0.1);
    EXPECT_NEAR(result[2], 0.0, 0.1);
}

TEST(ShotScopeTest, flightPath)
{
    const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
    const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

    GolfBallPhysicsVariables physVars(ball, atmos);
    GolfBallFlight flight(physVars, ball, atmos);

    Simulator simulator(physVars, flight);

    std::vector<Vector3D> result = simulator.runSimulation();

    // I think within 0.5 a foot is perfectly fine
    EXPECT_NEAR(result[result.size() - 3][0], 0.0, 0.5);
    EXPECT_NEAR(result[result.size() - 3][1], 805.688, 0.5);
    EXPECT_NEAR(result[result.size() - 3][2], 1.086, 0.5);

    EXPECT_NEAR(result[result.size() - 2][0], 0.0, 0.5);
    EXPECT_NEAR(result[result.size() - 2][1], 806.270, 0.5);
    EXPECT_NEAR(result[result.size() - 2][2], 0.509, 0.5);
}