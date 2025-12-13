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
 * @endcode
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#ifndef TERRAIN_INTERFACE_HPP
#define TERRAIN_INTERFACE_HPP

#include "ground_surface.hpp"
#include "math_utils.hpp"

#include <memory>

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
     * The normal vector points upward from the surface and has unit length.
     * For flat terrain, this is always (0, 0, 1).
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
     */
    explicit FlatTerrain(const GroundSurface& surface) : surface(surface) {}

    /**
     * Gets the terrain height (constant everywhere).
     *
     * @param x The x-coordinate (unused for flat terrain).
     * @param y The y-coordinate (unused for flat terrain).
     * @return The terrain height in feet.
     */
    [[nodiscard]] auto getHeight(float x, float y) const -> float override
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
    [[nodiscard]] auto getNormal(float x, float y) const -> Vector3D override
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
    [[nodiscard]] auto getSurfaceProperties(float x, float y) const -> const GroundSurface& override
    {
        // Suppress unused parameter warnings
        (void)x;
        (void)y;
        return surface;
    }

private:
    GroundSurface surface;
};

#endif // TERRAIN_INTERFACE_HPP
