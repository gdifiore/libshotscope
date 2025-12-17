# Usage Guide

## Installation

Run `build.sh` in the root directory to build and install the library. Include the main header in your source files:

```c++
#include <libgolf.hpp>
```

## Basic Setup

The library uses a phase-based flight simulation architecture that automatically transitions between aerial, bounce, and roll phases. A complete simulation requires initializing four components:

### 1. Ball and Atmospheric Parameters

```c++
const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};
```

Field definitions are documented in `include/golf_ball.hpp` and `include/atmosphere.hpp`. Parameter validation is the responsibility of the calling code. In production use, these values typically originate from launch monitors or environmental sensors.

### 2. Ground Surface Properties

#### Single Ground Surface (Simple)

```c++
GroundSurface ground; // Uses default fairway properties

// Or with custom values using constructor:
// GroundSurface green{0.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};
```

The `GroundSurface` struct defines physical surface characteristics that affect bounce and roll behavior. Default values represent typical fairway conditions. You can customize using the constructor: `GroundSurface{height, restitution, frictionStatic, frictionDynamic, firmness, spinRetention}`. See `include/ground_surface.hpp` for parameter details.

#### Dynamic Ground Surfaces (Advanced)

For simulations requiring different ground types throughout the trajectory (e.g., fairway → rough → green), implement the `GroundProvider` interface:

```c++
class MyGroundProvider : public GroundProvider {
public:
    GroundSurface getGroundAt(float x, float y) const override {
        // Return different surfaces based on position
        // x = lateral position (feet)
        // y = downrange position (feet)
    }
};

MyGroundProvider provider;
FlightSimulator sim(physVars, ball, atmos, provider);
```

See [Ground Providers Guide](ground_providers.md) for details.

### 3. Physics Variables

```c++
GolfBallPhysicsVariables physVars(ball, atmos);
```

This class computes derived aerodynamic coefficients from the ball and atmospheric parameters.

### 4. Flight Simulator

```c++
FlightSimulator sim(physVars, ball, atmos, ground);
```

The simulator manages phase transitions and numerical integration throughout the ball's trajectory.

## Initializing the Simulation

Create the initial ball state from sensor data. All ball parameters (position, velocity, spin) must be explicitly provided:

```c++
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
```

The `fromLaunchParameters()` factory method converts launch speed (ft/s), angle (degrees), and direction (degrees) into velocity components. The initial position must be converted from yards to feet. Spin magnitude is derived from the ball's backspin and sidespin values via `physVars.getROmega()`. Initialization is required before advancing the simulation.

## Running the Simulation

### Manual Stepping

Advance the simulation using a time-stepping loop:

```c++
const float dt = 0.01F; // 10ms time step

while (!sim.isComplete())
{
    sim.step(dt);
}

const BallState& finalState = sim.getState();
```

The `step()` method advances the simulation by one time increment and automatically handles phase transitions. The `isComplete()` method returns true when the ball has come to rest.

### Trajectory Collection

To capture the complete flight path for visualization or analysis:

```c++
std::vector<Vector3D> trajectory;
const float dt = 0.01F;

while (!sim.isComplete())
{
    const BallState& state = sim.getState();
    trajectory.push_back(state.position);
    sim.step(dt);
}

trajectory.push_back(sim.getState().position); // Final position
```

See `examples/calculate_ball_trajectory.cpp` for a complete implementation.

### Landing Point Only

For applications requiring only the final position:

```c++
const float dt = 0.01F;

while (!sim.isComplete())
{
    sim.step(dt);
}

const BallState& finalState = sim.getState();
Vector3D landingPosition = finalState.position;
```

See `examples/calculate_ball_landing.cpp` for a complete implementation.

## Coordinate System

The library uses a right-handed coordinate system:
- **x-axis**: Lateral direction (positive = right of target line)
- **y-axis**: Forward/downrange direction (direction = 0° points along +y)
- **z-axis**: Vertical/height (positive = up)

All position and velocity components are in feet. Use `physics_constants::YARDS_TO_FEET` for unit conversion when needed.

## Flight Phases

The simulator automatically manages three flight phases:

1. **Aerial**: Ball in flight subject to aerodynamic forces
2. **Bounce**: Ball impacting and rebounding from the ground surface
3. **Roll**: Ball rolling along the ground until coming to rest

Phase transitions occur automatically based on physical conditions. The current phase can be queried using `sim.getCurrentPhaseName()`.

## Advanced Features

### Dynamic Ground Surfaces

When using a `GroundProvider` instead of a single `GroundSurface`, the simulator automatically queries ground properties at the ball's current position during phase transitions. This enables realistic modeling of:

- Golf holes with fairways, roughs, and greens
- Elevated surfaces (e.g., raised greens)
- Varying terrain firmness and friction
- Bunkers and other hazards

Ground is queried before each bounce and periodically during rolling (every 0.1s), so the ball automatically picks up surface changes as it moves. See the [Ground Providers Guide](ground_providers.md) for details.

### Example Programs

The `examples/` directory contains complete working implementations:

- **calculate_ball_landing.cpp**: Compute final landing position only
- **calculate_ball_trajectory.cpp**: Collect full trajectory for visualization
- **multi_ground_simulation.cpp**: Demonstrate dynamic ground types (fairway/rough/green)