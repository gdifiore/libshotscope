/**
 * @file math_helpers.cpp
 * @author Gabriel DiFiore
 * @brief Contains functions for various mathematical calculations
 *
 * This file contains functions that do simple conversions (e.g. Fahrenheit to Celsius)
 * and more complex ones (e.g. calculate the Reynolds number).
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

// External includes
#include <cmath> // pow(), M_PI
// My includes
#include "math_helpers.hpp"
#include "math_constants.hpp"
#include "atmosphere.hpp"
#include "math_variables.hpp"
#include "golf_ball.hpp"

/* Conversion Functions */
/**
 * Converts Fahrenheit to Celsius.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Celsius.
 */
float convertFahrenheitToCelsius (float fahrenheit) {
    return (fahrenheit - 32) * 5 / 9;
}

/**
 * Converts Celsius to Kelvin.
 *
 * @param celsius The temperature in Celsius.
 * @return The temperature in Kelvin.
 */
float convertCelsiusToKelvin (float celsius) {
    return celsius + 273.15;
}

/**
 * Converts Fahrenheit to Kelvin.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Kelvin.
 */
float convertFarenheitToKelvin (float fahrenheit) {
    return ((fahrenheit - 32) * 5 / 9) + 273.15;
}

/* Calculating Intermediate Values */
/**
 * Calculates the air density value (in metric) for a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcRhoMetric (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos) {
    float kelvinTemp = convertFarenheitToKelvin(atmos->temp);
    float pressureFactor = atmos->pressure * std::exp(-vars->beta * vars->elevationM);
    float humidityFactor = 0.3783 * atmos->relHumidity * vars->SVP / 100;
    vars->rhoMetric = 1.2929 * (273 / kelvinTemp) * (pressureFactor - humidityFactor) / 760;
}

/**
 * Calculates the air density (in imperial) based on the metric density (rhoMetric).
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcRhoImperial (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos) {
    vars->rhoImperial = vars->rhoMetric * 0.06261;
}

/**
 * Calculates the value of c0 (damping coefficient) for a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcc0 (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos) {
    vars->c0 = 0.07182 * vars->rhoImperial * (5.125 / std_golf_ball_mass) * std::pow(std_golf_ball_circumference / 9.125, 2);
}

/**
 * Calculates the initial velocity in the x-direction (v0x) of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcv0x (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos) {
    vars->v0x = 1.467 * vars->v0 * std::cos(ball->launchAngle * M_PI / 180) * std::sin(ball->direction * M_PI / 180);
}

/**
 * Calculates the initial velocity of a golf ball.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcv0 (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos) {
    vars->v0 = ball->exitSpeed * 1.467;
}

/**
 * Calculates the Reynolds Number for a golf ball with velocity=100mph.
 * Used for determining which coefficient of drag to use.
 *
 * @param ball Struct containing current ball information.
 * @param vars Struct containing current variables for calculations.
 * @param atmos Struct containing current atmospheric information.
 */
void calcRe100 (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos) {

}

// move this eventually
void initVars (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos) {
    size_t numCalcFuncs = sizeof(calcFuncs) / sizeof(CalcFuncPtr);
    for (size_t i = 0; i < numCalcFuncs; ++i) {
        calcFuncs[i](ball, vars, atmos);
    }
}