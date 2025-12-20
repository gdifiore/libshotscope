#ifndef FLIGHT_SIMULATOR_HPP
#define FLIGHT_SIMULATOR_HPP

#include "BallState.hpp"
#include "FlightPhase.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "GroundProvider.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "ground_surface.hpp"
#include "terrain_interface.hpp"

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
	 * @param physicsVars Physics variables calculator (will be copied)
	 * @param ball Golf ball parameters
	 * @param atmos Atmospheric conditions
	 * @param ground Ground surface properties
	 * @param terrain Terrain interface for height/normal queries (optional, defaults to flat terrain)
	 */
	FlightSimulator(const GolfBallPhysicsVariables &physicsVars,
	                const golfBall &ball,
	                const atmosphericData &atmos,
	                const GroundSurface &ground,
	                std::shared_ptr<TerrainInterface> terrain = nullptr);

	/**
	 * @brief Constructs a flight simulator with a custom ground provider.
	 *
	 * This constructor allows for dynamic ground type changes during the trajectory.
	 * The ground provider is wrapped in a TerrainInterface adapter for compatibility.
	 *
	 * @param physicsVars Physics variables calculator (will be copied)
	 * @param ball Golf ball parameters
	 * @param atmos Atmospheric conditions
	 * @param groundProvider Ground provider for position-dependent ground properties
	 */
	FlightSimulator(const GolfBallPhysicsVariables &physicsVars,
	                const golfBall &ball,
	                const atmosphericData &atmos,
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

	// Physics variables storage (owned by this simulator)
	// Must be declared before phases since phases reference it
	GolfBallPhysicsVariables physicsVars_;

	// Terrain storage (for GroundProvider adapter lifetime management)
	// Must be declared before phases since phases depend on it
	std::shared_ptr<TerrainInterface> terrainStorage_;

	AerialPhase aerialPhase;
	BouncePhase bouncePhase;
	RollPhase rollPhase;

	/**
	 * @brief Checks if the current phase is complete and transitions if needed.
	 */
	void checkPhaseTransition();
};

#endif // FLIGHT_SIMULATOR_HPP
