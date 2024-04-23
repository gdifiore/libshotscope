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

/* Conversion Functions */
/**
 * Converts Fahrenheit to Celsius.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Celsius.
 */
float convertFahrenheitToCelsius(float fahrenheit) {
    return (fahrenheit - 32) * 5 / 9;
}

/**
 * Converts Celsius to Kelvin.
 *
 * @param celsius The temperature in Celsius.
 * @return The temperature in Kelvin.
 */
float convertCelsiusToKelvin(float celsius) {
    return celsius + 273.15;
}

/**
 * Converts Fahrenheit to Kelvin.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Kelvin.
 */
float convertFarenheitToKelvin(float fahrenheit) {
    return ((fahrenheit - 32) * 5 / 9) + 273.15;
}

/* Calculating Intermediate Values */

void calcRhoMetric(struct variables *vars, struct atmosphericData *atmos) {
    float kelvinTemp = convertFarenheitToKelvin(atmos->temp);
    float pressureFactor = atmos->pressure * std::exp(-vars->beta * vars->elevationM);
    float humidityFactor = 0.3783 * atmos->relHumidity * vars->SVP / 100;
    vars->rhoMetric = 1.2929 * (273 / kelvinTemp) * (pressureFactor - humidityFactor) / 760;
}

/**
 * Calculates the Reynolds Number for a golf ball with velocity=100mph.
 * Used for determining which coefficient of drag to use.
 *
 * @param atmos Struct containing current atmospheric information.
*/
void calcRe100(struct variables *vars, struct atmosphericData *atmos) {

}

// move this eventually
void initVarsStruct(struct variables *vars, struct atmosphericData *atmos) {
    size_t numCalcFuncs = sizeof(calcFuncs) / sizeof(CalcFuncPtr);
    for (size_t i = 0; i < numCalcFuncs; ++i) {
        calcFuncs[i](vars, atmos);
    }
}