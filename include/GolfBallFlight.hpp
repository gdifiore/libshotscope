#ifndef GOLFBALLFLIGHT_HPP
#define GOLFBALLFLIGHT_HPP

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

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

	// Getters
	float getV() const { return v; }
	float getVMph() const { return vMph; }
	const Vector3D &getVelocity3D() const { return velocity3D; }
	float getPhi() const { return phi; }
	float getTau() const { return tau; }
	float getRw() const { return rw; }
	float getWPerp() const { return w_perp; }
	float getWPerpDivW() const { return w_perp_div_w; }
	float getRe_x_e5() const { return Re_x_e5; }
	float getVw() const { return vw; }
	float getVwMph() const { return vwMph; }
	float getSpinFactor() const { return spinFactor; }
	const Vector3D &getVelocity3D_w() const { return velocity3D_w; }
	const Vector3D &getAccelerationDrag3D() const { return accelerationDrag3D; }
	const Vector3D &getAccelertaionMagnitude3D() const
	{
		return accelertaionMagnitude3D;
	}
	const Vector3D &getAcceleration3D() const { return acceleration3D; }

	const Vector3D &getPosition() const { return position; }

	float determineCoefficientOfDrag();
	float determineCoefficientOfLift();

	void calculateFlightStep();

private:
	GolfBallPhysicsVariables &physicsVars;
	struct golfBall ball;
	struct atmosphericData atmos;

	const float dt = 0.01f;
	float currentTime = 0.0f; // just needed for Rw

	// Member variables               // column title in excel sheet
	Vector3D position;
	Vector3D velocity3D;			  // vx/vy/vz
	float v;						  // v
	float vMph;						  // vmph
	float phi;						  // phi
	float tau;						  // tau
	float rw;						  // rw
	float w_perp = 0.0f;			  // w_perp
	float w_perp_div_w = 1.0f;		  // w_perp_div_w
	float Re_x_e5;					  // Re x e-5
	float vw;						  // vw
	float vwMph;					  // vwmph
	float spinFactor;				  // S
	Vector3D velocity3D_w;			  // vxw/vyw
	Vector3D accelerationDrag3D;	  // adragx/y/z
	Vector3D accelertaionMagnitude3D; // aMagx/y/z
	Vector3D acceleration3D;		  // ax/y/z

	// Private calculation methods (the order they're in is the order they need to be called)
	void initialize();

	void calculatePosition();
	void calculateV();
	void calculateVelocityw();
	void calculateVw();

	void calculatePhi();
	void calculateTau();
	void calculateRw();
	void calculateRe_x_e5();
	void calculateSpinFactor();

	void calculateAccelD();
	void calculateAccelM();
	void calculateAccel();
};

#endif // GOLFBALLFLIGHT_HPP