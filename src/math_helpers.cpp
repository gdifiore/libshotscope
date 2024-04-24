/**
 * @file math_helpers.cpp
 * @author Gabriel DiFiore
 * @brief Contains functions for various mathematical calculations
 *
 * This file contains functions that calculate intermediate variables for
 * calculating the flight of a golf ball.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

// External includes
#include <cmath> // pow(), M_PI
#include <stdio.h>
// My includes
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "math_constants.hpp"
#include "math_helpers.hpp"
#include "math_utils.hpp"
#include "math_variables.hpp"

// I want all of these to have the same signature to be used as function pointers, so this is fine.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
/**
 * Calculates the air density value (in metric) for a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcRhoMetric(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->rhoMetric = 1.2929 * ((273.0 / (vars->tempC + 273)) * ((vars->barometricPressure * exp(-vars->beta * vars->elevationM) - 0.3783 * atmos->relHumidity * (vars->SVP / 100.0)) / 760.0));
}

/**
 * Calculates the air density (in imperial) based on the metric density (rhoMetric).
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcRhoImperial(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->rhoImperial = vars->rhoMetric * 0.06261;
}

/**
 * Calculates the value of c0 (damping coefficient) for a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcc0(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->c0 = 0.07182 * vars->rhoImperial * (5.125 / std_golf_ball_mass) * std::pow(std_golf_ball_circumference / 9.125, 2);
}

/**
 * Calculates the initial velocity of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcv0(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->v0 = ball->exitSpeed * 1.467; // convert mph to m
}

/**
 * Calculates the initial velocity in the x-direction of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcv0x(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->v0x = 1.467 * ball->exitSpeed * std::cos(ball->launchAngle * M_PI / 180.0) * std::sin(ball->direction * M_PI / 180.0);
}

/**
 * Calculates the initial velocity in the y-direction of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcv0y(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->v0y = 1.467 * ball->exitSpeed * std::cos(ball->launchAngle * M_PI / 180.0) * std::cos(ball->direction * M_PI / 180.0);
}

/**
 * Calculates the initial velocity in the z-direction of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcv0z(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->v0z = 1.467 * ball->exitSpeed * std::sin(ball->launchAngle * M_PI / 180.0);
}

/**
 * Calculates the initial angular velocity in the x-direction of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcwx(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->wx = (ball->backspin * std::cos(ball->direction * M_PI / 180.0) - ball->sidespin * std::sin(ball->launchAngle * M_PI / 180.0) * std::sin(ball->direction * M_PI / 180.0)) * M_PI / 30.0;
}

/**
 * Calculates the initial angular velocity in the y-direction of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcwy(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->wy = (-ball->backspin * std::sin(ball->direction * M_PI / 180.0) - ball->sidespin * std::sin(ball->launchAngle * M_PI / 180.0) * std::cos(ball->direction * M_PI / 180.0)) * M_PI / 30.0;
}

/**
 * Calculates the initial angular velocity in the z-direction of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcwz(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->wz = (ball->sidespin * std::cos(ball->launchAngle * M_PI / 180.0)) * M_PI / 30.0;
}

/**
 * Calculates the initial spin (rad/s) of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcOmega(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->omega = std::sqrt(std::pow(ball->backspin, 2) + std::pow(ball->sidespin, 2)) * M_PI / 30.0;
}

/**
 * Calculates the initial linear velocity at the surface of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcROmega(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->rOmega = (std_golf_ball_circumference / (2 * M_PI)) * (vars->omega / 12.0);
}

/**
 * Calculates the crosswinds (ft/s) in x dir of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcvxw(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->vxw = atmos->vWind * 1.467 * std::sin(atmos->phiWind * M_PI / 180.0);
}

/**
 * Calculates the crosswinds (ft/s) in y dir of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcvyw(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->vyw = atmos->vWind * 1.467 * std::cos(atmos->phiWind * M_PI / 180.0);
}

/**
 * Calculates the Saturation Vapor Pressure (mmHg).
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcSVP(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->SVP = 4.5841 * exp((18.687 - vars->tempC / 234.5) * vars->tempC / (257.14 + vars->tempC));
}

/**
 * Calculates the Barometric Pressure (mmHg).
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcBarometricPressure(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->barometricPressure = atmos->pressure * 1000.0 / 39.37;
}

/**
 * Calculates the Reynolds Number for a golf ball with velocity=100mph.
 * Used for determining which coefficient of drag to use.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcRe100(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    vars->Re100 = vars->rhoMetric * 44.7 * (std_golf_ball_circumference / (M_PI * 39.37)) * (convertCelsiusToKelvin(vars->tempC) + 120)
                    / (0.000001512 * std::pow(convertCelsiusToKelvin(vars->tempC), 1.5));
}
#pragma GCC diagnostic pop

/**
 * Initialize all intermediate variables needed for physics calculations.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void initVars(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos)
{
    CalcFuncPtr calcFuncs[] = {
        calcBarometricPressure,
        calcSVP,
        calcOmega,
        calcROmega,
        calcRhoMetric,
        calcRhoImperial,
        calcc0,
        calcv0,
        calcv0x,
        calcv0y,
        calcv0z,
        calcwx,
        calcwy,
        calcwz,
        calcvxw,
        calcvyw,
        calcRe100
    };

    // these don't depend on any crazy physics calculations
    vars->beta = 0.0001217;
    vars->tempC = convertFahrenheitToCelsius(atmos->temp);
    vars->elevationM = convertFeetToMeters(atmos->elevation);

    uint8_t numCalcFuncs = sizeof(calcFuncs) / sizeof(CalcFuncPtr);
    for (uint8_t i = 0; i < numCalcFuncs; ++i)
    {
        calcFuncs[i](ball, vars, atmos);
    }
}