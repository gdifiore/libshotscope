#include <libshotscope.hpp>

#include <stdio.h>

int main()
{
    const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
    const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

    GolfBallPhysicsVariables physVars(ball, atmos);
    GolfBallFlight flight(physVars, ball, atmos);

    Simulator simulator(flight);

    std::vector<Vector3D> result = simulator.runSimulation();

    printf("Entire ball trajectory");

    for (const auto& entry : result)
    {
        printf("%.1f %.1f %.1f\n", entry[0]/math_utils::YARDS_TO_FEET, entry[1]/math_utils::YARDS_TO_FEET, entry[2]/math_utils::YARDS_TO_FEET);
    }

    return 0;
}