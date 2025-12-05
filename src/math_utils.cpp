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
            result[i] = vec[i] / physics_constants::YARDS_TO_FEET;
        }
        return result;
    };

    Vector3D last = divideBy3(positions.back());
    Vector3D second_last = divideBy3(positions[positions.size() - 2]);

    float heightDiff = last[2] / (last[2] - second_last[2]);

    Vector3D result;
    for (int i = 0; i < 3; ++i)
    {
        result[i] = last[i] - heightDiff * (last[i] - second_last[i]);
    }

    return result;
}

/**
 * Reflects a velocity vector off a surface defined by its normal vector.
 *
 * @param velocity The incoming velocity vector.
 * @param normal The surface normal vector (should be normalized).
 * @return The reflected velocity vector.
 */
Vector3D math_utils::reflect(const Vector3D& velocity, const Vector3D& normal)
{
    float dot = dotProduct(velocity, normal);
    Vector3D result;
    for (int i = 0; i < 3; ++i)
    {
        result[i] = velocity[i] - 2.0f * dot * normal[i];
    }
    return result;
}

/**
 * Projects a vector onto another vector.
 *
 * @param vector The vector to project.
 * @param onto The vector to project onto.
 * @return The projected vector.
 */
Vector3D math_utils::project(const Vector3D& vector, const Vector3D& onto)
{
    float dotVectorOnto = dotProduct(vector, onto);
    float dotOntoOnto = dotProduct(onto, onto);

    Vector3D result;
    for (int i = 0; i < 3; ++i)
    {
        result[i] = (dotVectorOnto / dotOntoOnto) * onto[i];
    }
    return result;
}

/**
 * Calculates the dot product of two vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The dot product.
 */
float math_utils::dotProduct(const Vector3D& a, const Vector3D& b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/**
 * Calculates the cross product of two vectors.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The cross product vector.
 */
Vector3D math_utils::crossProduct(const Vector3D& a, const Vector3D& b)
{
    Vector3D result;
    result[0] = a[1] * b[2] - a[2] * b[1];
    result[1] = a[2] * b[0] - a[0] * b[2];
    result[2] = a[0] * b[1] - a[1] * b[0];
    return result;
}