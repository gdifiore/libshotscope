#ifndef GROUND_SURFACE_HPP
#define GROUND_SURFACE_HPP

/**
 * @brief Represents the physical properties of the ground surface.
 *
 * This structure defines the characteristics of the ground that affect
 * ball behavior during bouncing and rolling phases.
 */
struct GroundSurface
{
	/**
	 * @brief Z-coordinate of the ground surface (in feet).
	 *
	 * Typically 0.0 for flat ground at origin level.
	 */
	float height = 0.0F;

	/**
	 * @brief Coefficient of restitution (COR) for bouncing.
	 *
	 * Determines how much energy is retained after a bounce.
	 * Range: 0.0 (no bounce) to 1.0 (perfect elastic bounce)
	 * Typical values:
	 * - Hard fairway: ~0.4-0.5
	 * - Soft rough: ~0.2-0.3
	 * - Green: ~0.3-0.4
	 */
	float restitution = 0.4F;

	/**
	 * @brief Coefficient of static friction.
	 *
	 * Affects initial impact and transition from bounce to roll.
	 * Range: 0.0 (no friction) to 1.0+ (high friction)
	 * Typical values:
	 * - Dry fairway: ~0.4-0.6
	 * - Wet grass: ~0.2-0.3
	 * - Green: ~0.3-0.5
	 */
	float frictionStatic = 0.5F;

	/**
	 * @brief Coefficient of dynamic (rolling) friction.
	 *
	 * Determines deceleration during rolling phase.
	 * Range: 0.0 (no friction) to 1.0+ (high friction)
	 * Typical values:
	 * - Fairway: ~0.15-0.25
	 * - Rough: ~0.4-0.6
	 * - Green: ~0.1-0.15
	 */
	float frictionDynamic = 0.2F;

	/**
	 * @brief Ground firmness coefficient.
	 *
	 * Represents how much the ball embeds into the surface on impact.
	 * Higher values = firmer ground (less embedding).
	 * Range: 0.0 (very soft) to 1.0+ (very firm)
	 * Typical values:
	 * - Hard fairway: ~0.9-1.0
	 * - Soft rough: ~0.3-0.5
	 * - Bunker: ~0.1-0.2
	 */
	float firmness = 0.8F;

	/**
	 * @brief Spin retention coefficient on impact.
	 *
	 * Determines how much spin is retained after ground impact.
	 * Range: 0.0 (all spin lost) to 1.0 (no spin lost)
	 * Typical values:
	 * - Hard fairway: ~0.7-0.8 (20-30% spin loss)
	 * - Soft rough: ~0.5-0.6 (40-50% spin loss)
	 * - Green: ~0.75-0.85 (15-25% spin loss)
	 */
	float spinRetention = 0.75F;

	/**
	 * @brief Default constructor with typical fairway values.
	 */
	GroundSurface() = default;

	/**
	 * @brief Constructs a GroundSurface with specified properties.
	 */
	GroundSurface(float h, float rest, float fStatic, float fDynamic, float firm, float spinRet = 0.75F)
		: height(h), restitution(rest), frictionStatic(fStatic),
		  frictionDynamic(fDynamic), firmness(firm), spinRetention(spinRet) {}
};

#endif // GROUND_SURFACE_HPP
