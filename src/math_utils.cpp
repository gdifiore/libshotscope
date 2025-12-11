/**
 * @file math_utils.cpp
 * @author Gabriel DiFiore
 * @brief Contains functions for various mathematical utilities.
 *
 * This file contains functions that perform mathematical
 * utilities, e.g. converting between units.
 *
 * @copyright Copyright (c) 2024, Gabriel DiFiore
 */

#include "math_utils.hpp"
#include "physics_constants.hpp"

#include <cmath>
#include <stdexcept>

/**
 * Converts Fahrenheit to Celsius.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Celsius.
 */
float math_utils::convertFahrenheitToCelsius(float fahrenheit)
{
    return (fahrenheit - physics_constants::FAHRENHEIT_OFFSET) * physics_constants::FAHRENHEIT_TO_CELSIUS_SCALE;
}

/**
 * Converts Celsius to Kelvin.
 *
 * @param celsius The temperature in Celsius.
 * @return The temperature in Kelvin.
 */
float math_utils::convertCelsiusToKelvin(float celsius)
{
    return celsius + physics_constants::KELVIN_OFFSET;
}

/**
 * Converts Fahrenheit to Kelvin.
 *
 * @param fahrenheit The temperature in Fahrenheit.
 * @return The temperature in Kelvin.
 */
float math_utils::convertFahrenheitToKelvin(float fahrenheit)
{
    return ((fahrenheit - physics_constants::FAHRENHEIT_OFFSET) * physics_constants::FAHRENHEIT_TO_CELSIUS_SCALE) + physics_constants::KELVIN_OFFSET;
}

/**
 * Converts feet to meters.
 *
 * @param feet The length in feet.
 * @return The length in meters.
 */
float math_utils::convertFeetToMeters(float feet)
{
    return feet * physics_constants::FEET_TO_METERS;
}

/**
 * Converts meters to feet.
 *
 * @param meters The length in meters.
 * @return The length in feet.
 */
float math_utils::convertMetersToFeet(float meters)
{
    return meters * physics_constants::METERS_TO_FEET;
}

/**
 * Calculates the distance in yards from the origin to the given landing position.
 *
 * @param position The position in 3D space.
 * @return The distance in yards.
 */
float math_utils::getDistanceInYards(Vector3D position)
{
    float distance = std::sqrt(position[0] * position[0] + position[1] * position[1]);

    float distanceInYards = distance / physics_constants::YARDS_TO_FEET;

    return distanceInYards;
}
