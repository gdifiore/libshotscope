#include "Simulator.hpp"
#include "golf_ball.hpp"
#include "atmosphere.hpp"
#include "math_utils.hpp"

int main()
{
    const golfBall ball {0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
    const atmosphericData atmos {70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

    GolfBallPhysicsVariables physVars(ball, atmos);
    GolfBallFlight flight(physVars, ball, atmos);

    Simulator simulator(physVars, flight);

    Vector3D result = simulator.runSimulation();
    printf("Distance: %f\n", math_utils::getDistanceInYards(result));

    return 0;
}