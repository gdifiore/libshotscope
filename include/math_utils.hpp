#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <array>
#include <vector>

using Vector3D = std::array<float, 3>;

namespace math_utils
{
    float convertFahrenheitToCelsius(float fahrenheit);
    float convertCelsiusToKelvin(float celsius);
    float convertFahrenheitToKelvin(float fahrenheit);

    constexpr float METERS_TO_FEET = 3.28084f;
    constexpr float FEET_TO_METERS = 1.0f / METERS_TO_FEET;
    constexpr float YARDS_TO_FEET = 3.0f;

    float convertFeetToMeters(float feet);
    float convertMetersToFeet(float meters);

    float getDistanceInYards(Vector3D position);

    Vector3D calcLandingPoint(const std::vector<Vector3D> &positions);
}

#endif // MATH_UTILS_HPP