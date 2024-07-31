#include "GolfBallFlightIntermediary.hpp"

#include <math.h>

#include "GolfBallKinematics.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "math_constants.hpp"

GolfBallFlightIntermediary::GolfBallFlightIntermediary(
    GolfBallPhysicsVariables &physicsVars, const struct golfBall &ball,
    const struct atmosphericData &atmos): physicsVars(physicsVars), ball(ball), atmos(atmos) {
  // Initialize position, velocity, and acceleration
  position = {0.0, 0.0, 0.0};
  velocity3D = {0.0, 0.0, 0.0};
  acceleration3D = {0.0, 0.0, 0.0};
  accelerationDrag3D = {0.0, 0.0, 0.0};
  accelertaionMagnitude3D = {0.0, 0.0, 0.0};

  // Set default values for other variables
  v = 0.0;
  vMph = 0.0;
  vw = 0.0;
  rw = 0.0;
  tau = 0.0;
  Re_x_e5 = 0.0;
}

void GolfBallFlightIntermediary::calculatePosition() {
  position[0] =
      position[0] + velocity3D[0] * d_t + 0.5 * acceleration3D[0] * d_t * d_t;
  position[1] =
      position[1] + velocity3D[1] * d_t + 0.5 * acceleration3D[1] * d_t * d_t;
  position[2] =
      position[2] + velocity3D[2] * d_t + 0.5 * acceleration3D[2] * d_t * d_t;
}

void GolfBallFlightIntermediary::calculateV() {
  float vx = velocity3D[0] + acceleration3D[0] * GolfBallKinematics::dt;
  float vy = velocity3D[1] + acceleration3D[1] * GolfBallKinematics::dt;
  float vz = velocity3D[2] + acceleration3D[2] * GolfBallKinematics::dt;

  velocity3D = {vx, vy, vz};

  v = sqrt(vx * vx + vy * vy + vz * vz);
  vMph = v / 1.467;
}

void GolfBallFlightIntermediary::calculateVelocityw() {}

void GolfBallFlightIntermediary::calculateAccel() {
  acceleration3D[0] = accelerationDrag3D[0] + accelertaionMagnitude3D[0];
  acceleration3D[1] = accelerationDrag3D[1] + accelertaionMagnitude3D[1];
  acceleration3D[2] =
      accelerationDrag3D[2] + accelertaionMagnitude3D[2] - 32.174;
}

void GolfBallFlightIntermediary::calculateAccelD() {
  accelerationDrag3D[0] = -physicsVars.getC0() * determineCoefficientOfDrag() *
                          velocity3D_w[0] * (velocity3D[0] - velocity3D_w[0]);
  accelerationDrag3D[1] = -physicsVars.getC0() * determineCoefficientOfDrag() *
                          velocity3D_w[1] * (velocity3D[1] - velocity3D_w[1]);
  accelerationDrag3D[2] = -physicsVars.getC0() * determineCoefficientOfDrag() *
                          velocity3D_w[2] * velocity3D[2];
}

void GolfBallFlightIntermediary::calculateAccelM() {
  accelertaionMagnitude3D[0] =
      physicsVars.getC0() *
      (determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
      (physicsVars.getW()[1] * velocity3D[2] -
       physicsVars.getW()[2] * (velocity3D[1] - velocity3D_w[1])) /
      w_perp_div_w;
  accelertaionMagnitude3D[1] =
      physicsVars.getC0() *
      (determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
      (physicsVars.getW()[2] * (velocity3D[1] - physicsVars.getVw()[1]) -
       physicsVars.getW()[0] * velocity3D[2]) /
      w_perp_div_w;
  accelertaionMagnitude3D[2] =
      physicsVars.getC0() *
      (determineCoefficientOfLift() / physicsVars.getOmega()) * vw *
      (physicsVars.getW()[0] * (velocity3D[1] - physicsVars.getVw()[1]) -
       physicsVars.getW()[1] * (velocity3D[0] - velocity3D_w[0])) /
      w_perp_div_w;
}

void GolfBallFlightIntermediary::calculateTau() {
  tau =
      1 / (0.00002 * v / (ball.std_golf_ball_circumference / (2 * M_PI * 12)));
}

void GolfBallFlightIntermediary::calculateRw() {
  rw = physicsVars.getROmega() * exp(-t_sec / tau);
}

void GolfBallFlightIntermediary::calculateVw() {
  if (position[2] >= atmos.hWind) {
    vw = sqrt(pow(velocity3D[0] - velocity3D_w[0], 2) +
              pow(velocity3D[1] - velocity3D_w[1], 2) + pow(velocity3D[2], 2));
  } else {
    vw = v;
  }
}

void GolfBallFlightIntermediary::calculateRe_x_e5() {
  Re_x_e5 = (vwMph / 100) * physicsVars.getRe100() * 0.00001;
}

float GolfBallFlightIntermediary::determineCoefficientOfDrag() {
  if (getRe_x_e5() <= math_constants::CdL) {
    return math_constants::CdL;
  } else if (getRe_x_e5() < 1) {
    return math_constants::CdL -
           (math_constants::CdL - math_constants::CdH) * (getRe_x_e5() - 0.5) /
               0.5 +
           math_constants::CdS * getSpinFactor();
  } else {
    return math_constants::CdH + math_constants::CdS * getSpinFactor();
  }
}

float GolfBallFlightIntermediary::determineCoefficientOfLift() {
  if (getSpinFactor() <= 0.3) {
    return math_constants::coeff1 * getSpinFactor() +
           math_constants::coeff2 * pow(getSpinFactor(), 2);
  } else {
    return 0.305;
  }
}

void GolfBallFlightIntermediary::calculateAllVariables() {
	calculateV();
	calculateVelocityw();
	calculateAccel();
	calculateAccelD();
	calculateAccelM();
	calculateTau();
	calculateRw();
	calculateVw();
	calculateRe_x_e5();
    calculatePosition();
}