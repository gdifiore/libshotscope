#ifndef GOLFBALLFLIGHT_HPP
#define GOLFBALLFLIGHT_HPP

#include <cmath>
#include <memory>

#include "BallState.hpp"
#include "FlightPhase.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "math_utils.hpp"

class GolfBallFlight
{
public:
	GolfBallFlight(GolfBallPhysicsVariables &physicsVars,
				   const struct golfBall &ball,
				   const struct atmosphericData &atmos);

	// Getters - delegate to aerialPhase
	[[nodiscard]] auto getV() const -> float { return aerialPhase->getV(); }
	[[nodiscard]] auto getVMph() const -> float { return aerialPhase->getVMph(); }
	[[nodiscard]] auto getVelocity3D() const -> const Vector3D &
	{
		return state.velocity;
	}
	[[nodiscard]] auto getPhi() const -> float { return aerialPhase->getPhi(); }
	[[nodiscard]] auto getTau() const -> float { return aerialPhase->getTau(); }
	[[nodiscard]] auto getRw() const -> float { return aerialPhase->getRw(); }
	[[nodiscard]] auto getWPerp() const -> float { return aerialPhase->getWPerp(); }
	[[nodiscard]] auto getWPerpDivW() const -> float
	{
		return aerialPhase->getWPerpDivW();
	}
	[[nodiscard]] auto getRe_x_e5() const -> float { return aerialPhase->getRe_x_e5(); }
	[[nodiscard]] auto getVw() const -> float { return aerialPhase->getVw(); }
	[[nodiscard]] auto getVwMph() const -> float { return aerialPhase->getVwMph(); }
	[[nodiscard]] auto getSpinFactor() const -> float { return aerialPhase->getSpinFactor(); }
	[[nodiscard]] auto getVelocity3D_w() const -> const Vector3D &
	{
		return aerialPhase->getVelocity3D_w();
	}
	[[nodiscard]] auto getAccelerationDrag3D() const -> const Vector3D &
	{
		return aerialPhase->getAccelerationDrag3D();
	}
	[[nodiscard]] auto getAccelertaionMagnitude3D() const
		-> const Vector3D &
	{
		return aerialPhase->getAccelertaionMagnitude3D();
	}
	[[nodiscard]] auto getAcceleration3D() const -> const Vector3D &
	{
		return state.acceleration;
	}
	[[nodiscard]] auto getPosition() const -> const Vector3D &
	{
		return state.position;
	}
	[[nodiscard]] auto getState() const -> const BallState &
	{
		return state;
	}

	auto determineCoefficientOfDrag() -> float;
	auto determineCoefficientOfLift() -> float;

	void calculateFlightStep();

private:
	GolfBallPhysicsVariables &physicsVars;
	struct golfBall ball;
	struct atmosphericData atmos;

	// Physics state (position, velocity, acceleration, time)
	BallState state;

	// Current flight phase (using AerialPhase)
	std::unique_ptr<AerialPhase> aerialPhase;

	// Private initialization method
	void initialize();
};

#endif // GOLFBALLFLIGHT_HPP