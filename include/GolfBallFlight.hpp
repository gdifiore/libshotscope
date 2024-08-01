#ifndef GOLFBALLFLIGHT_HPP
#define GOLFBALLFLIGHT_HPP

#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>

#include "GolfBallPhysicsVariables.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"

class GolfBallFlight
{
public:
	using Vector3D = std::array<float, 3>;

	GolfBallFlight(GolfBallPhysicsVariables &physicsVars,
				   const struct golfBall &ball,
				   const struct atmosphericData &atmos);

	GolfBallFlight &operator=(const GolfBallFlight &other)
	{
		if (this == &other)
			return *this;

		// Copy member variables
		physicsVars = other.physicsVars;
		ball = other.ball;
		atmos = other.atmos;
		dt = other.dt;
		t_sec = other.t_sec;
		position = other.position;
		velocity3D = other.velocity3D;
		v = other.v;
		vMph = other.vMph;
		phi = other.phi;
		tau = other.tau;
		rw = other.rw;
		w_perp = other.w_perp;
		w_perp_div_w = other.w_perp_div_w;
		Re_x_e5 = other.Re_x_e5;
		vw = other.vw;
		vwMph = other.vwMph;
		spinFactor = other.spinFactor;
		velocity3D_w = other.velocity3D_w;
		accelerationDrag3D = other.accelerationDrag3D;
		accelertaionMagnitude3D = other.accelertaionMagnitude3D;
		acceleration3D = other.acceleration3D;

		return *this;
	}

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

	// Calculate all variables
	void calculateFlightStep();

private:
	GolfBallPhysicsVariables &physicsVars;
	struct golfBall ball;
	struct atmosphericData atmos;

	float dt = 0.01f;	// dt
	float t_sec = 0.0f; // t_sec

	// Member variables               // column title in excel sheet
	Vector3D position;
	Vector3D velocity3D; // vx/vy/vz
	float v;			 // v
	float vMph;			 // vmph

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

	// just needed for Rw
	float currentTime = 0.0f;

	// Private calculation methods
	void initialize();

	void calculateV();
	void calculateVelocityw();

	void calculateAccel();
	void calculateAccelD();
	void calculateAccelM();

	void calculatePhi();
	void calculateTau();
	void calculateRw();
	void calculateVw();
	void calculateRe_x_e5();
	void calculateSpinFactor();
	void calculatePosition();
};

#endif // GOLFBALLFLIGHT_HPP