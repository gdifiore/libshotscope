#ifndef FLIGHT_SIMULATOR_HPP
#define FLIGHT_SIMULATOR_HPP

#include "BallState.hpp"
#include "FlightPhase.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "GroundProvider.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "ground_surface.hpp"
#include <memory>

/**
 * @brief Manages the complete flight simulation with automatic phase transitions.
 *
 * FlightSimulator encapsulates the complexity of managing different flight phases
 * (aerial, bounce, roll) and handles transitions automatically. Clients only need
 * to call step() repeatedly until the simulation is complete.
 *
 * IMPORTANT: You must call initialize() before calling step() or getState().
 *
 * Example usage:
 * @code
 * FlightSimulator sim(physicsVars, ball, atmos, ground);
 * sim.initialize(initialState);  // REQUIRED before calling step()
 * while (!sim.isComplete()) {
 *     sim.step(0.01f);
 * }
 * @endcode
 */
class FlightSimulator
{
public:
	/**
	 * @brief Constructs a flight simulator with the given parameters.
	 *
	 * This constructor uses a uniform ground surface (same properties everywhere).
	 * For backward compatibility with existing code.
	 *
	 * @param physicsVars Physics variables calculator
	 * @param ball Golf ball parameters
	 * @param atmos Atmospheric conditions
	 * @param ground Ground surface properties
	 */
	FlightSimulator(GolfBallPhysicsVariables &physicsVars,
					const struct golfBall &ball,
					const struct atmosphericData &atmos,
					const GroundSurface &ground);

	/**
	 * @brief Constructs a flight simulator with a custom ground provider.
	 *
	 * This constructor allows for dynamic ground type changes during the trajectory.
	 * The ground provider will be queried at phase transitions to get ground properties
	 * based on the ball's current XY position.
	 *
	 * @param physicsVars Physics variables calculator
	 * @param ball Golf ball parameters
	 * @param atmos Atmospheric conditions
	 * @param groundProvider Ground provider for position-dependent ground properties
	 */
	FlightSimulator(GolfBallPhysicsVariables &physicsVars,
					const struct golfBall &ball,
					const struct atmosphericData &atmos,
					const GroundProvider &groundProvider);

	/**
	 * @brief Initializes the simulation with the given initial state.
	 *
	 * This must be called before stepping through the simulation.
	 * Sets up the aerial phase with the provided initial conditions.
	 *
	 * @param initialState The initial ball state (position, velocity, etc.)
	 */
	void initialize(const BallState &initialState);

	/**
	 * @brief Advances the simulation by one time step.
	 *
	 * Must be called after initialize(). Calling before initialization will
	 * trigger an assertion failure in debug builds.
	 *
	 * Automatically handles phase transitions when the current phase completes.
	 * Safe to call even when simulation is complete (will have no effect).
	 *
	 * @param dt Time step duration in seconds
	 */
	void step(float dt);

	/**
	 * @brief Checks if the simulation has completed.
	 *
	 * @return true if the ball has stopped rolling, false otherwise
	 */
	[[nodiscard]] bool isComplete() const;

	/**
	 * @brief Gets the current ball state.
	 *
	 * Must be called after initialize().
	 *
	 * @return Reference to the current ball state
	 */
	[[nodiscard]] const BallState &getState() const;

	/**
	 * @brief Gets the name of the current flight phase.
	 *
	 * @return Phase name: "aerial", "bounce", "roll", or "complete"
	 */
	[[nodiscard]] const char *getCurrentPhaseName() const;

private:
	/**
	 * @brief Enumeration of flight phases.
	 */
	enum class Phase
	{
		Aerial,	 // Ball in flight
		Bounce,	 // Ball bouncing on ground
		Roll,	 // Ball rolling on ground
		Complete // Simulation finished
	};

	Phase currentPhase;
	BallState state;
	bool initialized;

	AerialPhase aerialPhase;
	BouncePhase bouncePhase;
	RollPhase rollPhase;

	// Ground provider support
	const GroundProvider *groundProvider;
	std::unique_ptr<UniformGroundProvider> uniformProvider;
	GroundSurface currentGround;
	int rollStepCounter;

	/**
	 * @brief Checks if the current phase is complete and transitions if needed.
	 */
	void checkPhaseTransition();

	/**
	 * @brief Updates the current ground surface based on ball position.
	 *
	 * Queries the ground provider at the specified XY position and updates
	 * all flight phases with the new ground properties.
	 *
	 * @param x Lateral position in feet
	 * @param y Forward/downrange position in feet
	 */
	void updateGroundAtPosition(float x, float y);
};

#endif // FLIGHT_SIMULATOR_HPP
