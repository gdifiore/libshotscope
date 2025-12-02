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

#include "GolfBallPhysicsVariables.hpp"
#include "golf_ball.hpp"
#include "atmosphere.hpp"
#include "math_utils.hpp"

#include <cmath>
#include <iostream>

/**
 * @brief Constructs a GolfBallPhysicsVariables object.
 *
 * This constructor initializes a GolfBallPhysicsVariables object with the given golf ball and atmospheric data.
 *
 * @param ball The golf ball structure containing relevant data.
 * @param atmos The atmospheric data structure containing relevant data.
 *
 * @note The user is responsible for validating the ball and atmos structures before passing them to this constructor.
 *       This class assumes that the input data is valid and within physically reasonable ranges.
 *       Passing invalid or out-of-range data may lead to unexpected behavior or incorrect calculations.
 */
GolfBallPhysicsVariables::GolfBallPhysicsVariables(const struct golfBall &ball, const struct atmosphericData &atmos)
    : ball(ball), atmos(atmos), beta(0.0001217f)
{
    tempC = math_utils::convertFahrenheitToCelsius(atmos.temp);
    elevationM = math_utils::convertFeetToMeters(atmos.elevation);

    calculateAllVariables();
}

void GolfBallPhysicsVariables::calculateAllVariables()
{
    calculateBarometricPressure();
    calculateSVP();
    calculateOmega();
    calculateROmega();
    calculateRhoMetric();
    calculateRhoImperial();
    calculateC0();
    calculateV0();
    calculateW();
    calculateVw();
    calculateRe100();
}

void GolfBallPhysicsVariables::calculateRhoMetric()
{
    rhoMetric = 1.2929 * ((273.0 / (math_utils::convertCelsiusToKelvin(tempC)) *
                           ((barometricPressure * std::exp(-beta * elevationM) - 0.3783 * atmos.relHumidity * (SVP / 100.0)) / 760.0)));
}

void GolfBallPhysicsVariables::calculateRhoImperial()
{
    rhoImperial = rhoMetric * 0.06261;
}

void GolfBallPhysicsVariables::calculateC0()
{
    c0 = 0.07182 * rhoImperial * (5.125 / ball.std_golf_ball_mass) *
         std::pow(ball.std_golf_ball_circumference / 9.125, 2);
}

void GolfBallPhysicsVariables::calculateV0()
{
    v0_magnitude = ball.exitSpeed * 1.467;
    float v0x = v0_magnitude * std::cos(ball.launchAngle * M_PI / 180.0) * std::sin(ball.direction * M_PI / 180.0);
    float v0y = v0_magnitude * std::cos(ball.launchAngle * M_PI / 180.0) * std::cos(ball.direction * M_PI / 180.0);
    float v0z = v0_magnitude * std::sin(ball.launchAngle * M_PI / 180.0);
    v0 = Vector3D{v0x, v0y, v0z};
}

void GolfBallPhysicsVariables::calculateW()
{
    float wx = (ball.backspin * std::cos(ball.direction * M_PI / 180.0) -
                ball.sidespin * std::sin(ball.launchAngle * M_PI / 180.0) * std::sin(ball.direction * M_PI / 180.0)) *
               M_PI / 30.0;
    float wy = (-ball.backspin * std::sin(ball.direction * M_PI / 180.0) -
                ball.sidespin * std::sin(ball.launchAngle * M_PI / 180.0) * std::cos(ball.direction * M_PI / 180.0)) *
               M_PI / 30.0;
    float wz = (ball.sidespin * std::cos(ball.launchAngle * M_PI / 180.0)) * M_PI / 30.0;
    w = Vector3D{wx, wy, wz};
}

void GolfBallPhysicsVariables::calculateOmega()
{
    omega = std::sqrt(std::pow(ball.backspin, 2) + std::pow(ball.sidespin, 2)) * M_PI / 30.0;
}

void GolfBallPhysicsVariables::calculateROmega()
{
    rOmega = (ball.std_golf_ball_circumference / (2 * M_PI)) * (omega / 12.0);
}

void GolfBallPhysicsVariables::calculateVw()
{
    float vxw = atmos.vWind * 1.467 * std::sin(atmos.phiWind * M_PI / 180.0);
    float vyw = atmos.vWind * 1.467 * std::cos(atmos.phiWind * M_PI / 180.0);
    vw = Vector3D{vxw, vyw, 0.0f};
}

void GolfBallPhysicsVariables::calculateSVP()
{
    SVP = 4.5841 * std::exp((18.687 - tempC / 234.5) * tempC / (257.14 + tempC));
}

void GolfBallPhysicsVariables::calculateBarometricPressure()
{
    barometricPressure = atmos.pressure * 1000.0 / 39.37;
}

void GolfBallPhysicsVariables::calculateRe100()
{
    Re100 = rhoMetric * 44.7 * (ball.std_golf_ball_circumference / (M_PI * 39.37)) *
            (math_utils::convertCelsiusToKelvin(tempC) + 120) /
            (0.000001512 * std::pow(math_utils::convertCelsiusToKelvin(tempC), 1.5));
}
