#include "Simulator.hpp"
#include "golf_ball.hpp"
#include "atmosphere.hpp"
#include "math_utils.hpp"

struct golfBall initBall(float a, float b, float c, float d, float e, float f,
                         float g, float h)
{
    struct golfBall testBall;

    testBall.x0 = a;
    testBall.y0 = b;
    testBall.z0 = c;
    testBall.exitSpeed = d;
    testBall.launchAngle = e;
    testBall.direction = f;
    testBall.backspin = g;
    testBall.sidespin = h;

    return testBall;
}

struct atmosphericData initAtmos(float a, float b, float c, float d, float e,
                                 float f, float g)
{
    struct atmosphericData atmosData;

    atmosData.temp = a;
    atmosData.elevation = b;
    atmosData.vWind = c;
    atmosData.phiWind = d;
    atmosData.hWind = e;
    atmosData.relHumidity = f;
    atmosData.pressure = g;

    return atmosData;
}

int main()
{
    const struct golfBall ball =
        initBall(0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0);
    const struct atmosphericData atmos =
        initAtmos(70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92);

    GolfBallPhysicsVariables physVars(ball, atmos);
    GolfBallFlight flight(physVars, ball, atmos);

    Simulator simulator(physVars, flight);

    Vector3D result = simulator.runSimulation();
    printf("Distance: %f\n", math_utils::getDistanceInYards(result));

    return 0;
}