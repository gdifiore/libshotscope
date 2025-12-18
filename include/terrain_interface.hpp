/**
 * @file terrain_interface.hpp
 * @author Gabriel DiFiore
 * @brief Defines the interface for terrain height and surface property queries.
 *
 * This file provides an abstract interface for querying terrain data,
 * allowing different terrain implementations (flat, heightmap, procedural, etc.)
 * to be used interchangeably in the flight simulation.
 *
 * Example usage:
 * @code
 * // Define a custom terrain implementation
 * class MyTerrain : public TerrainInterface {
 * public:
 *     MyTerrain(const GroundSurface& surface) : surface_(surface) {}
 *
 *     float getHeight(float x, float y) const override {
 *         // Return terrain height at position (x, y) in feet
 *         return computeHeightFromHeightmap(x, y);
 *     }
 *
 *     Vector3D getNormal(float x, float y) const override {
 *         // Return unit normal vector perpendicular to surface at (x, y)
 *         return computeNormalFromGradient(x, y);
 *     }
 *
 *     const GroundSurface& getSurfaceProperties(float x, float y) const override {
 *         // Return surface properties (friction, restitution, etc.) at (x, y)
 *         return surface_;
 *     }
 *
 * private:
 *     GroundSurface surface_;
 * };
 *
 * // Use custom terrain in flight simulation
 * auto terrain = std::make_shared<MyTerrain>(groundSurface);
 * FlightSimulator sim(physicsVars, ball, atmos, groundSurface, terrain);
 *
 * // Note: When a custom terrain is provided, the groundSurface parameter
 * // serves as a fallback for backward compatibility. The flight simulator
 * // will query terrain properties from the TerrainInterface implementation.
 * @endcode
 *
 * @copyright Copyright (c) 2025, Gabriel DiFiore
 */

#ifndef TERRAIN_INTERFACE_HPP
#define TERRAIN_INTERFACE_HPP

#include "ground_surface.hpp"
#include "math_utils.hpp"

#include <memory>
#include <limits>
#include <stdexcept>

/**
 * Abstract interface for terrain queries.
 *
 * Implementations of this interface provide terrain data to the flight simulation,
 * including height, surface normal, and material properties at any given position.
 */
class TerrainInterface
{
public:
    virtual ~TerrainInterface() = default;

    TerrainInterface(const TerrainInterface&) = delete;
    TerrainInterface& operator=(const TerrainInterface&) = delete;

    TerrainInterface(TerrainInterface&&) = delete;
    TerrainInterface& operator=(TerrainInterface&&) = delete;

    /**
     * Gets the terrain height at the given horizontal position.
     *
     * @param x The x-coordinate (lateral position in feet).
     * @param y The y-coordinate (forward position in feet).
     * @return The terrain height (z-coordinate) in feet.
     */
    [[nodiscard]] virtual auto getHeight(float x, float y) const -> float = 0;

    /**
     * Gets the surface normal vector at the given horizontal position.
     *
     * The normal vector points upward from the surface (away from solid terrain,
     * into the air) and has unit length. It is perpendicular to the tangent plane
     * of the surface at the given position. For a horizontal flat surface, this is
     * always (0, 0, 1). For sloped surfaces, the normal tilts accordingly while
     * maintaining unit length.
     *
     * @param x The x-coordinate (lateral position in feet).
     * @param y The y-coordinate (forward position in feet).
     * @return The unit normal vector pointing upward from the surface.
     */
    [[nodiscard]] virtual auto getNormal(float x, float y) const -> Vector3D = 0;

    /**
     * Gets the surface properties at the given horizontal position.
     *
     * @param x The x-coordinate (lateral position in feet).
     * @param y The y-coordinate (forward position in feet).
     * @return The ground surface properties (restitution, friction, etc.).
     */
    [[nodiscard]] virtual auto getSurfaceProperties(float x, float y) const -> const GroundSurface& = 0;

protected:
    TerrainInterface() = default;
};

/**
 * Flat terrain implementation with uniform height and properties.
 *
 * This implementation provides backward compatibility with the original
 * flat ground assumption. The entire terrain has a single height and
 * uniform surface properties.
 */
class FlatTerrain : public TerrainInterface
{
public:
    /**
     * Constructs a flat terrain with the given surface properties.
     *
     * @param surface The ground surface properties to use everywhere.
     * @throws std::invalid_argument if surface properties are out of valid ranges.
     */
    explicit FlatTerrain(const GroundSurface& surface) : surface(surface)
    {
        // Validate surface properties are in physically meaningful ranges
        if (surface.restitution < 0.0F || surface.restitution > 1.0F)
        {
            throw std::invalid_argument("Restitution must be in range [0.0, 1.0]");
        }
        if (surface.frictionStatic < 0.0F)
        {
            throw std::invalid_argument("Static friction must be non-negative");
        }
        if (surface.frictionDynamic < 0.0F)
        {
            throw std::invalid_argument("Dynamic friction must be non-negative");
        }
        if (surface.firmness < 0.0F)
        {
            throw std::invalid_argument("Firmness must be non-negative");
        }
        if (surface.spinRetention < 0.0F || surface.spinRetention > 1.0F)
        {
            throw std::invalid_argument("Spin retention must be in range [0.0, 1.0]");
        }
    }

    /**
     * Gets the terrain height (constant everywhere).
     *
     * @param x The x-coordinate (unused for flat terrain).
     * @param y The y-coordinate (unused for flat terrain).
     * @return The terrain height in feet.
     */
    [[nodiscard]] auto getHeight(float x, float y) const noexcept -> float override
    {
        // Suppress unused parameter warnings
        (void)x;
        (void)y;
        return surface.height;
    }

    /**
     * Gets the surface normal (always vertical for flat terrain).
     *
     * @param x The x-coordinate (unused for flat terrain).
     * @param y The y-coordinate (unused for flat terrain).
     * @return The unit normal vector (0, 0, 1).
     */
    [[nodiscard]] auto getNormal(float x, float y) const noexcept -> Vector3D override
    {
        // Suppress unused parameter warnings
        (void)x;
        (void)y;
        return {0.0F, 0.0F, 1.0F};
    }

    /**
     * Gets the surface properties (constant everywhere).
     *
     * @param x The x-coordinate (unused for flat terrain).
     * @param y The y-coordinate (unused for flat terrain).
     * @return The ground surface properties.
     */
    [[nodiscard]] auto getSurfaceProperties(float x, float y) const noexcept -> const GroundSurface& override
    {
        // Suppress unused parameter warnings
        (void)x;
        (void)y;
        return surface;
    }

private:
    GroundSurface surface;
};

// Forward declaration for GroundProvider
class GroundProvider;

/**
 * Adapter that wraps a GroundProvider to work as a TerrainInterface.
 *
 * This adapter allows the GroundProvider system (position-dependent ground properties)
 * to work with the TerrainInterface system. It assumes flat terrain with varying
 * surface properties based on position.
 *
 * The adapter owns a copy of the provider to ensure proper lifetime management.
 *
 * @warning This class is NOT thread-safe. It uses internal caching that will
 *          cause data races if accessed concurrently from multiple threads.
 *          Each thread should maintain its own instance.
 */
class TerrainProviderAdapter : public TerrainInterface
{
public:
	/**
	 * Constructs an adapter from a GroundProvider.
	 *
	 * Creates a copy of the provider to ensure it remains valid for the
	 * lifetime of this adapter.
	 *
	 * @param provider The ground provider to wrap (will be cloned)
	 * @throws std::invalid_argument if provider is null
	 */
	explicit TerrainProviderAdapter(const GroundProvider* provider);

	[[nodiscard]] auto getHeight(float x, float y) const -> float override;
	[[nodiscard]] auto getNormal(float x, float y) const noexcept -> Vector3D override;
	[[nodiscard]] auto getSurfaceProperties(float x, float y) const -> const GroundSurface& override;

private:
	std::unique_ptr<GroundProvider> provider_;
	mutable GroundSurface cachedSurface_;  // Mutable for caching in const method
	mutable float cachedX_ = 0.0F;
	mutable float cachedY_ = 0.0F;
	mutable bool cacheValid_ = false;  // Explicit initialization flag

	// Helper to update cache only when position changes
	void updateCache(float x, float y) const;
};

#endif // TERRAIN_INTERFACE_HPP
