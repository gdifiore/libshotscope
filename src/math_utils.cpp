/**
 * @file math_utils.cpp
 * @author Gabriel DiFiore
 * @brief Contains functions for various mathematical utilities
 *
 * This file contains functions that perform mathematical
 * utilities, e.g. converting between units.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#include <cmath>

#include "math_utils.hpp"

/**
 * Converts Fahrenheit to Celsius.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Celsius.
 */
float math_utils::convertFahrenheitToCelsius(float fahrenheit)
{
    return (fahrenheit - 32) * 5 / 9;
}

/**
 * Converts Celsius to Kelvin.
 *
 * @param celsius The temperature in Celsius.
 * @return The temperature in Kelvin.
 */
float math_utils::convertCelsiusToKelvin(float celsius)
{
    return celsius + 273.16;
}

/**
 * Converts Fahrenheit to Kelvin.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Kelvin.
 */
float math_utils::convertFahrenheitToKelvin(float fahrenheit)
{
    return ((fahrenheit - 32) * 5 / 9) + 273.16;
}

/**
 * Converts feet to meters.
 *
 * @param feet The length in feet.
 * @return The length in meters.
 */
float math_utils::convertFeetToMeters(float feet)
{
    return feet * FEET_TO_METERS;
}

/**
 * Converts meters to feet.
 *
 * @param meters The length in meters.
 * @return The length in feet.
 */
float math_utils::convertMetersToFeet(float meters)
{
    return meters * METERS_TO_FEET;
}

/**
 * Calculates the distance in yards from the origin to the given position.
 *
 * @param position The position in 3D space.
 * @return The distance in yards.
 */
float math_utils::getDistanceInYards(Vector3D position)
{
    float distance = std::sqrt(position[0] * position[0] + position[1] * position[1]);

    float distanceInYards = distance / YARDS_TO_FEET;

    return distanceInYards;
}
