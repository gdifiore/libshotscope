#ifndef GOLFBALLPHYSICSVARIABLES_HPP
#define GOLFBALLPHYSICSVARIABLES_HPP

#include <array>
#include <stdexcept>

#include "atmosphere.hpp"
#include "golf_ball.hpp"

class GolfBallPhysicsVariables {
 public:
  using Vector3D = std::array<float, 3>;

  GolfBallPhysicsVariables(const struct golfBall &ball,
                           const struct atmosphericData &atmos);

  // Getters
  float getRhoImperial() const { return rhoImperial; }
  float getRhoMetric() const { return rhoMetric; }
  float getC0() const { return c0; }
  float getBeta() const { return beta; }
  float getV0() const { return v0_magnitude; }
  Vector3D getV0Vector() const { return v0; }
  Vector3D getW() const { return w; }
  float getOmega() const { return omega; }
  float getROmega() const { return rOmega; }
  float getTempC() const { return tempC; }
  float getElevationM() const { return elevationM; }
  Vector3D getVw() const { return vw; }
  float getSVP() const { return SVP; }
  float getBarometricPressure() const { return barometricPressure; }
  float getRe100() const { return Re100; }

  // Calculate all variables
  void calculateAllVariables();

 private:
  struct golfBall ball;
  struct atmosphericData atmos;

  // Member variables
  float rhoImperial = 0.0f;
  float rhoMetric = 0.0f;
  float c0 = 0.0f;
  const float beta = 0.0001217f;
  float tempC = 0.0f;
  float elevationM = 0.0f;
  float v0_magnitude;

  Vector3D v0;
  Vector3D w;
  Vector3D vw;
  float omega;
  float rOmega;
  float SVP;
  float barometricPressure;
  float Re100;

  // Private calculation methods
  void calculateRhoMetric();
  void calculateRhoImperial();
  void calculateC0();
  void calculateV0();
  void calculateW();
  void calculateOmega();
  void calculateROmega();
  void calculateVw();
  void calculateSVP();
  void calculateBarometricPressure();
  void calculateRe100();
};

#endif  // GOLFBALLPHYSICSVARIABLES_HPP