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
 * Calculates the distance in yards from the origin to the given landing position.
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

/**
 * Calculates the landing point based on the flight path.
 *
 * @note Uses the last 2 points (right before passing through z=0 and right after).
 *
 * @param positions A vector of Vector3D positions.
 * @return The calculated landing point (yds) as a Vector3D.
 * @throws std::runtime_error if there are not enough positions to calculate the landing point.
 */
Vector3D math_utils::calcLandingPoint(const std::vector<Vector3D> &positions)
{
    if (positions.size() < 2)
    {
        throw std::runtime_error("Not enough positions to calculate landing point");
    }

    const auto divideBy3 = [](const Vector3D &vec)
    {
        Vector3D result;
        for (int i = 0; i < 3; ++i)
        {
            result[i] = vec[i] / 3.0F; // yards to feet
        }
        return result;
    };

    Vector3D last = divideBy3(positions.back());
    Vector3D second_last = divideBy3(positions[positions.size() - 2]);

    float heightDiff = last[2] / (last[2] - second_last[2]) / YARDS_TO_FEET;

    Vector3D result;
    for (int i = 0; i < 3; ++i)
    {
        result[i] = last[i] - heightDiff * (last[i] - second_last[i]);
    }

    return result;
}