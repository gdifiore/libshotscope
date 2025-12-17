# libshotscope
`libshotscope` is a C++ library designed to simulate golf ball trajectories based on initial conditions like velocity, atmospheric data, and more. It provides easy-to-use functions to visualize or calculate the ball's flight path and landing point (now including bounces, rolling, and dynamic ground surfaces!).

The in-air math here is based on [work done](http://baseball.physics.illinois.edu/trajectory-calculator-golf.html) by Prof. Alan M. Nathan at the  University of Illinois Urbana-Champaign.

## Build
```bash
git clone https://github.com/gdifiore/libshotscope.git

cd libshotscope

chmod +x build.sh

./build.sh
```

## Features

- Full trajectory simulation with automatic phase transitions (aerial → bounce → roll)
- Dynamic ground surfaces - fairways, roughs, greens, elevation changes
- Efficient step-by-step numerical integration

## Documentation

- [Getting Started](/docs/how.md) - Basic usage
- [Ground Providers](/docs/ground_providers.md) - Dynamic ground surfaces (fairways, roughs, greens)

### Quick Example

```cpp
#include <libshotscope.hpp>

// Basic simulation with single ground type
const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};
GroundSurface ground;  // Default fairway (or use constructor for custom values)

GolfBallPhysicsVariables physVars(ball, atmos);
FlightSimulator sim(physVars, ball, atmos, ground);

// Initialize and run
sim.initialize(initialState);
while (!sim.isComplete()) {
    sim.step(0.01f);
}
```

For dynamic ground surfaces (fairway/rough/green), see the [Ground Providers Guide](/docs/ground_providers.md).
