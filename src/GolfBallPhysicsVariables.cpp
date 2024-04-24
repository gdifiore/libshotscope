#include <cmath>

#include "GolfBallPhysicsVariables.hpp"
#include "golf_ball.hpp"
#include "atmosphere.hpp"
#include "math_utils.hpp"

GolfBallPhysicsVariables::GolfBallPhysicsVariables(const golfBall& ball, const atmosphericData& atmos)
    : ball(ball), atmos(atmos), beta(0.0001217), tempC(convertFahrenheitToCelsius(atmos.temp)), elevationM(convertFeetToMeters(atmos.elevation))
{
}

void GolfBallPhysicsVariables::calculateAllVariables() {
    calculateBarometricPressure();
    calculateSVP();
    calculateOmega();
    calculateROmega();
    calculateRhoMetric();
    calculateRhoImperial();
    calculateC0();
    calculateV0();
    calculateV0x();
    calculateV0y();
    calculateV0z();
    calculateWx();
    calculateWy();
    calculateWz();
    calculateVxw();
    calculateVyw();
    calculateRe100();
}

/**
 * Calculates the air density value (in metric) for a golf ball.
 */
void GolfBallPhysicsVariables::calculateRhoMetric() {
    rhoMetric = 1.2929 * ((273.0 / (tempC + 273)) * ((barometricPressure * exp(-beta * elevationM) - 0.3783 * atmos.relHumidity * (this->SVP / 100.0)) / 760.0));
}

void GolfBallPhysicsVariables::calculateRhoImperial() {
    rhoImperial = rhoMetric * 0.06261;
}

void GolfBallPhysicsVariables::calculateC0() {
    c0 = 0.07182 * rhoImperial * (5.125 / ball.std_golf_ball_mass) * std::pow(ball.std_golf_ball_circumference / 9.125, 2);
}

void GolfBallPhysicsVariables::calculateV0() {
    v0 = ball.exitSpeed * 1.467;
}

void GolfBallPhysicsVariables::calculateV0x() {
    v0x = 1.467 * ball.exitSpeed * std::cos(ball.launchAngle * M_PI / 180.0) * std::sin(ball.direction * M_PI / 180.0);
}

void GolfBallPhysicsVariables::calculateV0y() {
    v0y = 1.467 * ball.exitSpeed * std::cos(ball.launchAngle * M_PI / 180.0) * std::cos(ball.direction * M_PI / 180.0);
}

void GolfBallPhysicsVariables::calculateV0z() {
    v0z = 1.467 * ball.exitSpeed * std::sin(ball.launchAngle * M_PI / 180.0);
}

void GolfBallPhysicsVariables::calculateWx() {
    wx = (ball.backspin * std::cos(ball.direction * M_PI / 180.0) - ball.sidespin * std::sin(ball.launchAngle * M_PI / 180.0) * std::sin(ball.direction * M_PI / 180.0)) * M_PI / 30.0;
}

void GolfBallPhysicsVariables::calculateWy() {
    wy = (-ball.backspin * std::sin(ball.direction * M_PI / 180.0) - ball.sidespin * std::sin(ball.launchAngle * M_PI / 180.0) * std::cos(ball.direction * M_PI / 180.0)) * M_PI / 30.0;
}

void GolfBallPhysicsVariables::calculateWz() {
    wz = (ball.sidespin * std::cos(ball.launchAngle * M_PI / 180.0)) * M_PI / 30.0;
}

void GolfBallPhysicsVariables::calculateOmega() {
    omega = std::sqrt(std::pow(ball.backspin, 2) + std::pow(ball.sidespin, 2)) * M_PI / 30.0;
}

void GolfBallPhysicsVariables::calculateROmega() {
    rOmega = (ball.std_golf_ball_circumference / (2 * M_PI)) * (omega / 12.0);
}

void GolfBallPhysicsVariables::calculateVxw() {
    vxw = atmos.vWind * 1.467 * std::sin(atmos.phiWind * M_PI / 180.0);
}

void GolfBallPhysicsVariables::calculateVyw() {
    vyw = atmos.vWind * 1.467 * std::cos(atmos.phiWind * M_PI / 180.0);
}

void GolfBallPhysicsVariables::calculateSVP() {
    SVP = 4.5841 * exp((18.687 - tempC / 234.5) * tempC / (257.14 + tempC));
}

void GolfBallPhysicsVariables::calculateBarometricPressure() {
    barometricPressure = atmos.pressure * 1000.0 / 39.37;
}

void GolfBallPhysicsVariables::calculateRe100() {
    Re100 = rhoMetric * 44.7 * (ball.std_golf_ball_circumference / (M_PI * 39.37)) * (convertCelsiusToKelvin(tempC) + 120)
            / (0.000001512 * std::pow(convertCelsiusToKelvin(tempC), 1.5));
}