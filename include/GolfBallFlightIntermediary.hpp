#ifndef GOLFBALLFLIGHTINTERMEDIARY_HPP
#define GOLFBALLFLIGHTINTERMEDIARY_HPP

#include <array>
#include <optional>
#include <stdexcept>

#include "GolfBallPhysicsVariables.hpp"
#include "golf_ball.hpp"
#include "atmosphere.hpp"

class GolfBallFlightIntermediary {
 public:
  using Vector3D = std::array<float, 3>;

  GolfBallFlightIntermediary(GolfBallPhysicsVariables &physicsVars,
                             const struct golfBall &ball,
                             const struct atmosphericData &atmos);

  // Getters
  float getV() const { return v; }
  float getVMph() const { return vMph; }
  const Vector3D &getVelocity3D() const { return velocity3D; }
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
  const Vector3D &getAccelertaionMagnitude3D() const {
    return accelertaionMagnitude3D;
  }
  const Vector3D &getAcceleration3D() const { return acceleration3D; }

  // Want below to be in its own Kinematics Class
  const Vector3D &getPosition() const { return position; }

  // Want below to be in its own CoefficientModel class
  float determineCoefficientOfDrag();
  float determineCoefficientOfLift();

  // Calculate all variables
  void calculateAllVariables();

 private:
  GolfBallPhysicsVariables &physicsVars;
  struct golfBall ball;
  struct atmosphericData atmos;
  // CoefficientModel &coeffModel;

  // kinematics class
  Vector3D position = {ball.x0, ball.y0, ball.z0};

  // want this refactored out
  float d_t = 0.01f;  // dt
  float t_sec = 0.0f;  // t_sec

  // Member variables               // column title in excel sheet
  float v;                                            // v
  float vMph;                                         // vmph
  Vector3D velocity3D = {physicsVars.getV0Vector()};  // vx/vy/vz

  float tau;                         // tau
  float rw;                          // rw
  float w_perp = 0.0f;               // w_perp
  float w_perp_div_w = 1.0f;         // w_perp_div_w
  float Re_x_e5;                     // Re x e-5
  float vw;                          // vw
  float vwMph;                       // vwmph
  float spinFactor;                  // S
  Vector3D velocity3D_w;             // vxw/vyw
  Vector3D accelerationDrag3D;       // adragx/y/z
  Vector3D accelertaionMagnitude3D;  // aMagx/y/z
  Vector3D acceleration3D;           // ax/y/z

  // Private calculation methods
  void calculateV();
  void calculateVelocityw();

  void calculateAccel();
  void calculateAccelD();
  void calculateAccelM();

  void calculateTau();
  void calculateRw();
  void calculateVw();
  void calculateRe_x_e5();

  // Want below to be in its own Kinematics Class
  void calculatePosition();
};

#endif  // GOLFBALLFLIGHTINTERMEDIARY_HPP