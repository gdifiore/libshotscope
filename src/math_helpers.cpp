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
#include "math_constants.hpp"
#include "atmosphere.hpp"

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

/**
 * Calculates the Reynolds Number for a golf ball with velocity=100mph.
 * Used for determining which coefficient of drag to use.
 *
 * @param atmos Struct containing current atmospheric information.
 * @return The calculated Re-100 value.
*/
float calculateReynoldsNumber(struct atmosphericData *atmos) {

}