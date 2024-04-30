#ifndef GOLFBALLPHYSICSVARIABLES_HPP
#define GOLFBALLPHYSICSVARIABLES_HPP

#include "golf_ball.hpp"
#include "atmosphere.hpp"

class GolfBallPhysicsVariables
{
public:
    GolfBallPhysicsVariables(const golfBall &ball, const atmosphericData &atmos);

    void calculateAllVariables();

    float getRhoImperial() const { return rhoImperial; }
    float getRhoMetric() const { return rhoMetric; }
    float getC0() const { return c0; }
    float getBeta() const { return beta; }
    float getV0() const { return v0; }
    float getV0x() const { return v0x; }
    float getV0y() const { return v0y; }
    float getV0z() const { return v0z; }
    float getWx() const { return wx; }
    float getWy() const { return wy; }
    float getWz() const { return wz; }
    float getOmega() const { return omega; }
    float getROmega() const { return rOmega; }
    float getTempC() const { return tempC; }
    float getElevationM() const { return elevationM; }
    float getVxw() const { return vxw; }
    float getVyw() const { return vyw; }
    float getSVP() const { return SVP; }
    float getBarometricPressure() const { return barometricPressure; }
    float getRe100() const { return Re100; }

private:
    const golfBall &ball;
    const atmosphericData &atmos;

    float rhoImperial;
    float rhoMetric;
    float c0;
    float beta;
    float v0;
    float v0x;
    float v0y;
    float v0z;
    float wx;
    float wy;
    float wz;
    float omega;
    float rOmega;
    float tempC;
    float elevationM;
    float vxw;
    float vyw;
    float SVP;
    float barometricPressure;
    float Re100;

    void calculateRhoMetric();
    void calculateRhoImperial();
    void calculateC0();
    void calculateV0();
    void calculateV0x();
    void calculateV0y();
    void calculateV0z();
    void calculateWx();
    void calculateWy();
    void calculateWz();
    void calculateOmega();
    void calculateROmega();
    void calculateVxw();
    void calculateVyw();
    void calculateSVP();
    void calculateBarometricPressure();
    void calculateRe100();
};

#endif // GOLFBALLPHYSICSVARIABLES_HPP