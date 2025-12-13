/**
 * @file ground_physics.hpp
 * @author Gabriel DiFiore
 * @brief Ground interaction physics for golf ball bouncing and rolling.
 *
 * This module provides physics calculations for ball-ground interactions,
 * including slope-aware bouncing, rolling, and spin effects. The calculations
 * use the surface normal to properly handle sloped terrain.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#ifndef GROUND_PHYSICS_HPP
#define GROUND_PHYSICS_HPP

#include "ground_surface.hpp"
#include "math_utils.hpp"

namespace GroundPhysics
{
    /**
     * Result of a bounce calculation.
     */
    struct BounceResult
    {
        Vector3D newVelocity;  ///< Velocity after bounce
        float newSpinRate;     ///< Spin rate after bounce (rad/s)
    };

    /**
     * Calculates the bounce physics when the ball impacts the ground.
     *
     * This function decomposes the velocity into components normal and tangent
     * to the surface, applies the coefficient of restitution to the normal
     * component, and applies friction to the tangent component.
     *
     * @param velocity The velocity before bounce (ft/s).
     * @param surfaceNormal The unit normal vector of the surface (pointing upward).
     * @param spinRate The spin rate before bounce (rad/s).
     * @param surface The ground surface properties (restitution, friction, etc.).
     * @return The bounce result containing new velocity and spin rate.
     */
    [[nodiscard]] auto calculateBounce(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float spinRate,
        const GroundSurface& surface
    ) -> BounceResult;

    /**
     * Calculates the acceleration for a rolling ball on a slope.
     *
     * This function computes the net acceleration from:
     * - Gravity component along the slope
     * - Rolling friction opposing motion
     *
     * @param velocity The current velocity (ft/s).
     * @param surfaceNormal The unit normal vector of the surface (pointing upward).
     * @param spinRate The current spin rate (rad/s).
     * @param surface The ground surface properties (friction, etc.).
     * @return The acceleration vector (ft/s²).
     */
    [[nodiscard]] auto calculateRollAcceleration(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float spinRate,
        const GroundSurface& surface
    ) -> Vector3D;

    /**
     * Determines if the ball should transition from bouncing to rolling.
     *
     * The ball transitions to rolling when it's close to the ground and
     * moving slowly in the vertical direction.
     *
     * @param velocity The current velocity (ft/s).
     * @param surfaceNormal The unit normal vector of the surface (pointing upward).
     * @param heightAboveGround The height above the terrain surface (ft).
     * @return True if the ball should transition to rolling, false otherwise.
     */
    [[nodiscard]] auto shouldTransitionToRoll(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float heightAboveGround
    ) -> bool;

} // namespace GroundPhysics

#endif // GROUND_PHYSICS_HPP
