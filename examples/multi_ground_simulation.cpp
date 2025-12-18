/**
 * @file multi_ground_simulation.cpp
 * @brief Example demonstrating dynamic ground type changes during trajectory.
 *
 * This example shows how to use the GroundProvider interface to model a golf hole
 * with different ground surfaces: fairway, rough, and an elevated green.
 *
 * The simulated hole layout:
 * - Fairway: 0-250 yards (standard properties)
 * - Rough: Lateral edges beyond ±20 yards from centerline (higher friction, softer)
 * - Green: 250-270 yards, elevated 3 feet (low friction, high spin retention)
 */

#include "FlightSimulator.hpp"
#include "GroundProvider.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <stdio.h>
#include <vector>

/**
 * @brief Custom ground provider for a golf hole with fairway, rough, and green.
 *
 * This class demonstrates how to implement position-dependent ground surfaces.
 * Ground properties change based on the ball's XY position during the simulation.
 */
class GolfHoleGroundProvider : public GroundProvider
{
public:
	GolfHoleGroundProvider() = default;
	GolfHoleGroundProvider(const GolfHoleGroundProvider&) : GroundProvider() {}

	GroundSurface getGroundAt(float x, float y) const override
	{
		// Convert positions to yards for easier reasoning
		const float lateralYards = x / physics_constants::YARDS_TO_FEET;
		const float downrangeYards = y / physics_constants::YARDS_TO_FEET;

		// Green: 250-270 yards downrange, elevated 3 feet
		if (downrangeYards >= 250.0F && downrangeYards <= 270.0F)
		{
			return GroundSurface{
				3.0F,   // height: elevated 3 feet
				0.35F,  // restitution: lower bounce on green
				0.4F,   // frictionStatic: moderate
				0.12F,  // frictionDynamic: low for fast greens
				0.95F,  // firmness: very firm
				0.85F   // spinRetention: high (soft greens retain spin)
			};
		}

		// Rough: beyond ±20 yards from centerline
		if (std::abs(lateralYards) > 20.0F)
		{
			return GroundSurface{
				0.0F,   // height: ground level
				0.25F,  // restitution: softer bounce
				0.6F,   // frictionStatic: higher impact friction
				0.5F,   // frictionDynamic: much higher rolling resistance
				0.4F,   // firmness: softer ground
				0.55F   // spinRetention: lower (thick grass kills spin)
			};
		}

		// Fairway: default area (centerline, before green)
		return GroundSurface{
			0.0F,   // height: ground level
			0.4F,   // restitution: standard bounce
			0.5F,   // frictionStatic: moderate
			0.2F,   // frictionDynamic: standard rolling resistance
			0.8F,   // firmness: firm fairway
			0.75F   // spinRetention: good spin retention
		};
	}

	std::unique_ptr<GroundProvider> clone() const override
	{
		return std::make_unique<GolfHoleGroundProvider>(*this);
	}
};

int main()
{
	// Shot parameters: 160 mph ball speed, 11° launch angle, straight shot
	const golfBall ball{0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0};
	const atmosphericData atmos{70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92};

	// Create custom ground provider for our golf hole
	GolfHoleGroundProvider groundProvider;

	GolfBallPhysicsVariables physVars(ball, atmos);
	FlightSimulator sim(physVars, ball, atmos, groundProvider);

	// Setup initial state from sensor data
	const float v0_fps = ball.exitSpeed * physics_constants::MPH_TO_FT_PER_S;
	Vector3D start_pos{
		ball.x0 * physics_constants::YARDS_TO_FEET,
		ball.y0 * physics_constants::YARDS_TO_FEET,
		ball.z0 * physics_constants::YARDS_TO_FEET};

	BallState initialState = BallState::fromLaunchParameters(
		v0_fps,
		ball.launchAngle,
		ball.direction,
		start_pos,
		physics_constants::GRAVITY_FT_PER_S2,
		physVars.getROmega());

	sim.initialize(initialState);

	// Run simulation and collect trajectory points
	std::vector<Vector3D> trajectory;
	std::vector<const char *> phaseNames;
	const float dt = 0.01F;

	while (!sim.isComplete())
	{
		const BallState &state = sim.getState();
		trajectory.push_back(state.position);
		phaseNames.push_back(sim.getCurrentPhaseName());
		sim.step(dt);
	}

	// Add final position
	trajectory.push_back(sim.getState().position);
	phaseNames.push_back(sim.getCurrentPhaseName());

	// Print summary
	const BallState &finalState = sim.getState();
	const float finalLateralYards = finalState.position[0] / physics_constants::YARDS_TO_FEET;
	const float finalDownrangeYards = finalState.position[1] / physics_constants::YARDS_TO_FEET;
	const float finalHeight = finalState.position[2];

	printf("=== Multi-Ground Trajectory Simulation ===\n\n");

	printf("Shot Parameters:\n");
	printf("  Ball speed: %.1f mph\n", ball.exitSpeed);
	printf("  Launch angle: %.1f°\n", ball.launchAngle);
	printf("  Backspin: %.1f rpm\n\n", ball.backspin);

	printf("Final Landing:\n");
	printf("  Lateral: %.1f yards\n", finalLateralYards);
	printf("  Downrange: %.1f yards\n", finalDownrangeYards);
	printf("  Height: %.1f feet\n\n", finalHeight);

	// Determine final surface type
	const char *finalSurface;
	if (finalDownrangeYards >= 250.0F && finalDownrangeYards <= 270.0F)
	{
		finalSurface = "Green (elevated)";
	}
	else if (std::abs(finalLateralYards) > 20.0F)
	{
		finalSurface = "Rough";
	}
	else
	{
		finalSurface = "Fairway";
	}

	printf("Final Surface: %s\n\n", finalSurface);

	printf("Trajectory points (lateral, downrange, height in yards/feet):\n");
	printf("Phase     Lateral    Downrange  Height\n");
	printf("-------   --------   ---------  ------\n");

	// Print every 10th point to keep output manageable
	for (size_t i = 0; i < trajectory.size(); i += 10)
	{
		const auto &pos = trajectory[i];
		const char *phase = phaseNames[i];
		printf("%-8s  %8.1f   %9.1f  %6.1f\n",
		       phase,
		       pos[0] / physics_constants::YARDS_TO_FEET,
		       pos[1] / physics_constants::YARDS_TO_FEET,
		       pos[2]);
	}

	// Always print final point
	if (trajectory.size() % 10 != 1)
	{
		const auto &pos = trajectory.back();
		printf("%-8s  %8.1f   %9.1f  %6.1f\n",
		       phaseNames.back(),
		       pos[0] / physics_constants::YARDS_TO_FEET,
		       pos[1] / physics_constants::YARDS_TO_FEET,
		       pos[2]);
	}

	printf("\n");
	printf("Note: The ground type changes automatically as the ball\n");
	printf("      moves across different surfaces during the trajectory.\n");

	return 0;
}
