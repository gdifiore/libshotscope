/**
 * @file GroundProvider.hpp
 * @author Gabriel DiFiore
 * @brief Interface for querying ground surface properties at any position.
 *
 * This file defines the GroundProvider interface that enables dynamic ground
 * type changes throughout a golf ball's trajectory. Users can implement custom
 * ground mappings to model fairways, roughs, greens, and other surfaces with
 * varying properties and heights.
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#ifndef GROUND_PROVIDER_HPP
#define GROUND_PROVIDER_HPP

#include "ground_surface.hpp"
#include <memory>

/**
 * @brief Interface for querying ground surface properties at any XY position.
 *
 * This abstract class allows users to define custom ground surface mappings
 * based on ball position. The library will query the ground provider only when
 * needed (during phase transitions and ground contact), not continuously.
 *
 * Users can implement this interface to model:
 * - Fairways, roughs, greens with different properties
 * - Elevated surfaces (hills, elevated greens)
 * - Bunkers with different firmness and friction
 * - Terrain variations across the course
 *
 * Thread Safety: Implementations must be const-safe and thread-safe for
 * concurrent read access if the same provider instance is shared across
 * multiple simulators.
 *
 * Example implementation:
 * @code
 * class MyGroundProvider : public GroundProvider {
 * public:
 *     GroundSurface getGroundAt(float x, float y) const override {
 *         // Return green properties for distances > 250 yards
 *         if (y > 250.0f * YARDS_TO_FEET) {
 *             return GroundSurface{3.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};
 *         }
 *         // Return fairway properties otherwise
 *         return GroundSurface{0.0f, 0.4f, 0.5f, 0.2f, 0.8f, 0.75f};
 *     }
 * };
 * @endcode
 */
class GroundProvider
{
public:
	virtual ~GroundProvider() = default;

	GroundProvider(const GroundProvider&) = delete;
	GroundProvider& operator=(const GroundProvider&) = delete;

	GroundProvider(GroundProvider&&) = delete;
	GroundProvider& operator=(GroundProvider&&) = delete;

	/**
	 * @brief Query ground surface properties at a specific XY position.
	 *
	 * This method is called by the simulator at strategic points:
	 * - During aerial phase to check ground height for phase completion
	 * - At bounce impact to get surface properties (restitution, friction)
	 * - At start of roll phase to get rolling friction
	 *
	 * The implementation should be efficient as it may be called multiple times
	 * during a simulation (typically 3-10 times for a full trajectory).
	 *
	 * Exception Safety: Implementations should not throw exceptions. If the
	 * position is invalid or out of bounds, return a reasonable default surface
	 * rather than throwing.
	 *
	 * @param x Lateral position in feet (perpendicular to target line)
	 * @param y Forward/downrange position in feet (along target line)
	 * @return GroundSurface properties at the specified location
	 */
	virtual GroundSurface getGroundAt(float x, float y) const = 0;

	/**
	 * @brief Creates a copy of this ground provider.
	 *
	 * This method is used internally to ensure proper lifetime management when
	 * the provider is wrapped in a TerrainInterface adapter.
	 *
	 * @return A unique pointer to a cloned copy of this provider
	 */
	virtual std::unique_ptr<GroundProvider> clone() const = 0;

protected:
	GroundProvider() = default;
};

/**
 * @brief Simple ground provider that returns the same surface everywhere.
 *
 * This is the default implementation used for backward compatibility.
 * It returns the same ground surface properties regardless of position,
 * which is equivalent to the original single-ground behavior.
 *
 * This provider is automatically created when using the traditional
 * FlightSimulator constructor that takes a single GroundSurface.
 *
 * Example usage:
 * @code
 * GroundSurface green{0.0f, 0.35f, 0.4f, 0.12f, 0.95f, 0.85f};
 * UniformGroundProvider provider(green);
 * FlightSimulator sim(physicsVars, ball, atmos, provider);
 * @endcode
 */
class UniformGroundProvider : public GroundProvider
{
public:
	/**
	 * @brief Constructs a uniform ground provider.
	 *
	 * @param surface The ground surface to return for all positions
	 */
	explicit UniformGroundProvider(const GroundSurface &surface)
		: surface_(surface)
	{
	}

	/**
	 * @brief Returns the same ground surface for any position.
	 *
	 * @param x Lateral position (ignored)
	 * @param y Forward position (ignored)
	 * @return The ground surface specified at construction
	 */
	GroundSurface getGroundAt(float x, float y) const override
	{
		(void)x; // Unused parameter
		(void)y; // Unused parameter
		return surface_;
	}

	/**
	 * @brief Creates a copy of this uniform ground provider.
	 *
	 * @return A unique pointer to a cloned copy
	 */
	std::unique_ptr<GroundProvider> clone() const override
	{
		return std::make_unique<UniformGroundProvider>(surface_);
	}

private:
	GroundSurface surface_;
};

#endif // GROUND_PROVIDER_HPP
