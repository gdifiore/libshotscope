#ifndef GOLFBALLFLIGHT_HPP
#define GOLFBALLFLIGHT_HPP

#include <cmath>

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
        [[nodiscard]] auto getV() const -> float { return v; }
        [[nodiscard]] auto getVMph() const -> float { return vMph; }
        [[nodiscard]] auto getVelocity3D() const -> const Vector3D& {
          return velocity3D;
        }
        [[nodiscard]] auto getPhi() const -> float { return phi; }
        [[nodiscard]] auto getTau() const -> float { return tau; }
        [[nodiscard]] auto getRw() const -> float { return rw; }
        [[nodiscard]] auto getWPerp() const -> float { return w_perp; }
        [[nodiscard]] auto getWPerpDivW() const -> float {
          return w_perp_div_w;
        }
        [[nodiscard]] auto getRe_x_e5() const -> float { return Re_x_e5; }
        [[nodiscard]] auto getVw() const -> float { return vw; }
        [[nodiscard]] auto getVwMph() const -> float { return vwMph; }
        [[nodiscard]] auto getSpinFactor() const -> float { return spinFactor; }
        [[nodiscard]] auto getVelocity3D_w() const -> const Vector3D& {
          return velocity3D_w;
        }
        [[nodiscard]] auto getAccelerationDrag3D() const -> const Vector3D& {
          return accelerationDrag3D;
        }
        [[nodiscard]] auto getAccelertaionMagnitude3D() const
            -> const Vector3D& {
          return accelertaionMagnitude3D;
        }
        [[nodiscard]] auto getAcceleration3D() const -> const Vector3D& {
          return acceleration3D;
        }
        [[nodiscard]] auto getPosition() const -> const Vector3D& {
          return position;
        }

        auto determineCoefficientOfDrag() -> float;
        auto determineCoefficientOfLift() -> float;

        void calculateFlightStep();

private:
	GolfBallPhysicsVariables &physicsVars;
	struct golfBall ball;
	struct atmosphericData atmos;

	const float dt = 0.01F;
	float currentTime = 0.0F; // just needed for Rw

	// Member variables               // column title in excel sheet
	Vector3D position;
	Vector3D velocity3D;			  // vx/vy/vz
	float v;						  // v
	float vMph;						  // vmph
	float phi;						  // phi
	float tau;						  // tau
	float rw;						  // rw
	float w_perp = 0.0F;			  // w_perp
	float w_perp_div_w = 1.0F;		  // w_perp_div_w
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