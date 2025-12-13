/**
 * @file ground_physics.cpp
 * @author Gabriel DiFiore
 * @brief Implementation of ground interaction physics.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#include "ground_physics.hpp"
#include "physics_constants.hpp"

#include <cmath>

namespace GroundPhysics
{
    BounceResult calculateBounce(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float spinRate,
        const GroundSurface& surface)
    {
        BounceResult result;

        // Decompose velocity into normal and tangent components
        // v_normal = (v · n) * n
        // v_tangent = v - v_normal
        float velocityDotNormal = math_utils::dot(velocity, surfaceNormal);
        Vector3D velocityNormal = {
            surfaceNormal[0] * velocityDotNormal,
            surfaceNormal[1] * velocityDotNormal,
            surfaceNormal[2] * velocityDotNormal
        };

        Vector3D velocityTangent = {
            velocity[0] - velocityNormal[0],
            velocity[1] - velocityNormal[1],
            velocity[2] - velocityNormal[2]
        };

        // Apply coefficient of restitution to normal component
        // v'_normal = -COR * v_normal (reversed direction)
        Vector3D velocityNormalAfter = {
            -surface.restitution * velocityNormal[0],
            -surface.restitution * velocityNormal[1],
            -surface.restitution * velocityNormal[2]
        };

        // Apply friction to tangent component
        // Friction reduces tangent velocity based on surface properties
        float frictionFactor = 1.0F - surface.frictionStatic * (1.0F - surface.firmness);
        Vector3D velocityTangentAfter = {
            velocityTangent[0] * frictionFactor,
            velocityTangent[1] * frictionFactor,
            velocityTangent[2] * frictionFactor
        };

        // Combine components to get final velocity
        result.newVelocity = {
            velocityNormalAfter[0] + velocityTangentAfter[0],
            velocityNormalAfter[1] + velocityTangentAfter[1],
            velocityNormalAfter[2] + velocityTangentAfter[2]
        };

        // Apply spin retention
        result.newSpinRate = spinRate * surface.spinRetention;

        return result;
    }

    Vector3D calculateRollAcceleration(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float spinRate,
        const GroundSurface& surface)
    {
        // Suppress unused parameter warning (may be used in future enhancements)
        (void)spinRate;

        Vector3D acceleration = {0.0F, 0.0F, 0.0F};

        // Get horizontal velocity magnitude
        float vHorizontal = std::sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);

        // Avoid division by zero
        if (vHorizontal < physics_constants::MIN_VELOCITY_THRESHOLD)
        {
            return acceleration;
        }

        // Calculate slope angle from normal
        // cos(θ) = n · [0,0,1] = n_z
        float cosTheta = surfaceNormal[2];

        // For very flat surfaces, use simplified calculation
        constexpr float FLAT_SURFACE_THRESHOLD = 0.999F;  // ~2.5 degrees
        if (cosTheta > FLAT_SURFACE_THRESHOLD)
        {
            // Nearly flat surface: only rolling friction opposes motion
            float deceleration = surface.frictionDynamic * physics_constants::GRAVITY_FT_PER_S2;
            acceleration[0] = -deceleration * (velocity[0] / vHorizontal);
            acceleration[1] = -deceleration * (velocity[1] / vHorizontal);
            acceleration[2] = 0.0F;
            return acceleration;
        }

        // Calculate gravity vector
        Vector3D gravity = {0.0F, 0.0F, -physics_constants::GRAVITY_FT_PER_S2};

        // Decompose gravity into normal and tangent components
        float gravityDotNormal = math_utils::dot(gravity, surfaceNormal);
        Vector3D gravityNormal = {
            surfaceNormal[0] * gravityDotNormal,
            surfaceNormal[1] * gravityDotNormal,
            surfaceNormal[2] * gravityDotNormal
        };

        Vector3D gravityTangent = {
            gravity[0] - gravityNormal[0],
            gravity[1] - gravityNormal[1],
            gravity[2] - gravityNormal[2]
        };

        // Gravity component along slope (down-slope acceleration)
        acceleration[0] = gravityTangent[0];
        acceleration[1] = gravityTangent[1];
        acceleration[2] = gravityTangent[2];

        // Add rolling friction (opposes motion)
        float normalForce = std::abs(gravityDotNormal);
        float frictionDeceleration = surface.frictionDynamic * normalForce;

        acceleration[0] -= frictionDeceleration * (velocity[0] / vHorizontal);
        acceleration[1] -= frictionDeceleration * (velocity[1] / vHorizontal);

        return acceleration;
    }

    bool shouldTransitionToRoll(
        const Vector3D& velocity,
        const Vector3D& surfaceNormal,
        float heightAboveGround)
    {
        // Check if ball is close to the ground
        if (heightAboveGround > physics_constants::GROUND_CONTACT_THRESHOLD)
        {
            return false;
        }

        // Calculate velocity component normal to surface
        float velocityDotNormal = math_utils::dot(velocity, surfaceNormal);

        // Transition to roll if moving slowly in the normal direction
        return std::abs(velocityDotNormal) < physics_constants::MIN_BOUNCE_VELOCITY;
    }

} // namespace GroundPhysics
