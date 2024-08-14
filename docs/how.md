# How to Use `libshotscope`

To get started with the library, make sure you have run `build.sh` in the root directory to install it. Then, include the following header in your program:

```c++
#include <libshotscope.hpp>
```

Every program using the library should begin with the same 3-5 lines of code:

```c++
const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

GolfBallPhysicsVariables physVars(ball, atmos);
GolfBallFlight flight(physVars, ball, atmos);

Simulator simulator(flight);
```

Typically, the ball and atmospheric data would be initialized dynamically by physical sensors or chosen through another program, such as simulating weather conditions at Cypress Point. The `libshotscope` library focuses on calculating the flight path of the ball based on these input values.

Once you have an instance of the `Simulator` class, you can call one of two functions.

```c++
    auto runSimulation() -> std::vector<Vector3D>;
    auto runSimulationLanding() -> Vector3D;
```

where `Vector3D` is defined as `using Vector3D = std::array<float, 3>;`

The first function `runSimulation()` will return 3D location every 0.01s of the simulated flight. The last two points in the vector are the points just before and just after the ball goes through the ground. You can calculate the final landing spot using

```c++
Vector3D math_utils::calcLandingPoint(const std::vector<Vector3D> &positions)
```

This is meant for use with a shot tracer to visualize the path.

See `examples\calculate_ball_trajectory.cpp`.

The second function `runSimulationLanding()` will return just the final landing spot of the ball.

See `examples\calculate_ball_landing.cpp`