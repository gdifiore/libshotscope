/**
 * @file GolfBallPhysicsVariables.cpp
 * @author Gabriel DiFiore
 * @brief Contains the implementation of the GolfBallPhysicsVariables class.
 *
 * This file defines the GolfBallPhysicsVariables class, which is responsible for calculating
 * various intermediate physics variables related to a golf ball. It includes functions for
 * calculating air density, barometric pressure, spin rates, launch angles, wind effects, and
 * other variables used in golf ball physics calculations.
 *
 * The GolfBallPhysicsVariables class takes a golf ball object and atmospheric data as input,
 * and provides methods to calculate all the required variables.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#include <cmath>

#include "math_utils.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "golf_ball.hpp"
#include "atmosphere.hpp"

GolfBallPhysicsVariables::GolfBallPhysicsVariables(const golfBall& ball, const atmosphericData& atmos)
    : ball(ball), atmos(atmos), beta(0.0001217)
{
    tempC = math_utils::convertFahrenheitToCelsius(atmos.temp);
    elevationM = math_utils::convertFeetToMeters(atmos.elevation);
}

/**
 * Initialize all intermediate variables needed for physics calculations.
 */
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
    rhoMetric = 1.2929 * ((273.0 / (math_utils::convertCelsiusToKelvin(tempC)) * ((barometricPressure * exp(-beta * elevationM) - 0.3783 * atmos.relHumidity * (this->SVP / 100.0)) / 760.0)));
}
/**
 * Calculates the air density (in imperial) based on the metric density (rhoMetric).
 */
void GolfBallPhysicsVariables::calculateRhoImperial() {
    rhoImperial = rhoMetric * 0.06261;
}

/**
 * Calculates the value of c0 (damping coefficient) for a golf ball.
 */
void GolfBallPhysicsVariables::calculateC0() {
    c0 = 0.07182 * rhoImperial * (5.125 / ball.std_golf_ball_mass) * std::pow(ball.std_golf_ball_circumference / 9.125, 2);
}

/**
 * Calculates the initial velocity (m/s) of a golf ball.
 */
void GolfBallPhysicsVariables::calculateV0() {
    v0 = ball.exitSpeed * 1.467;
}

/**
 * Calculates the initial velocity in the x-direction of a golf ball.
 */
void GolfBallPhysicsVariables::calculateV0x() {
    v0x = 1.467 * ball.exitSpeed * std::cos(ball.launchAngle * M_PI / 180.0) * std::sin(ball.direction * M_PI / 180.0);
}

/**
 * Calculates the initial velocity in the y-direction of a golf ball.
 */
void GolfBallPhysicsVariables::calculateV0y() {
    v0y = 1.467 * ball.exitSpeed * std::cos(ball.launchAngle * M_PI / 180.0) * std::cos(ball.direction * M_PI / 180.0);
}

/**
 * Calculates the initial velocity in the z-direction of a golf ball.
 */
void GolfBallPhysicsVariables::calculateV0z() {
    v0z = 1.467 * ball.exitSpeed * std::sin(ball.launchAngle * M_PI / 180.0);
}

/**
 * Calculates the initial angular velocity in the x-direction of a golf ball.
 */
void GolfBallPhysicsVariables::calculateWx() {
    wx = (ball.backspin * std::cos(ball.direction * M_PI / 180.0) - ball.sidespin * std::sin(ball.launchAngle * M_PI / 180.0) * std::sin(ball.direction * M_PI / 180.0)) * M_PI / 30.0;
}

/**
 * Calculates the initial angular velocity in the y-direction of a golf ball.
 */
void GolfBallPhysicsVariables::calculateWy() {
    wy = (-ball.backspin * std::sin(ball.direction * M_PI / 180.0) - ball.sidespin * std::sin(ball.launchAngle * M_PI / 180.0) * std::cos(ball.direction * M_PI / 180.0)) * M_PI / 30.0;
}

/**
 * Calculates the initial angular velocity in the z-direction of a golf ball.
 */
void GolfBallPhysicsVariables::calculateWz() {
    wz = (ball.sidespin * std::cos(ball.launchAngle * M_PI / 180.0)) * M_PI / 30.0;
}

/**
 * Calculates the initial spin (rad/s) of a golf ball.
 */
void GolfBallPhysicsVariables::calculateOmega() {
    omega = std::sqrt(std::pow(ball.backspin, 2) + std::pow(ball.sidespin, 2)) * M_PI / 30.0;
}


/**
 * Calculates the initial linear velocity at the surface of a golf ball.
 */
void GolfBallPhysicsVariables::calculateROmega() {
    rOmega = (ball.std_golf_ball_circumference / (2 * M_PI)) * (omega / 12.0);
}

/**
 * Calculates the crosswinds (ft/s) in x dir of a golf ball.
 */
void GolfBallPhysicsVariables::calculateVxw() {
    vxw = atmos.vWind * 1.467 * std::sin(atmos.phiWind * M_PI / 180.0);
}


/**
 * Calculates the crosswinds (ft/s) in y dir of a golf ball.
 */
void GolfBallPhysicsVariables::calculateVyw() {
    vyw = atmos.vWind * 1.467 * std::cos(atmos.phiWind * M_PI / 180.0);
}

/**
 * Calculates the Saturation Vapor Pressure (mmHg).
 */
void GolfBallPhysicsVariables::calculateSVP() {
    SVP = 4.5841 * exp((18.687 - tempC / 234.5) * tempC / (257.14 + tempC));
}

/**
 * Calculates the Barometric Pressure (mmHg).
 */
void GolfBallPhysicsVariables::calculateBarometricPressure() {
    barometricPressure = atmos.pressure * 1000.0 / 39.37;
}

/**
 * Calculates the Reynolds Number for a golf ball with velocity=100mph.
 * Used for determining which coefficient of drag to use.
 */
void GolfBallPhysicsVariables::calculateRe100() {
    Re100 = rhoMetric * 44.7 * (ball.std_golf_ball_circumference / (M_PI * 39.37)) * (math_utils::convertCelsiusToKelvin(tempC) + 120)
            / (0.000001512 * std::pow(math_utils::convertCelsiusToKelvin(tempC), 1.5));
}