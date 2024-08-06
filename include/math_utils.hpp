#ifndef MATH_UTILS_HPP
#define MATH_UTILS_HPP

#include <array>
#include <vector>

using Vector3D = std::array<float, 3>;

namespace math_utils
{
    auto convertFahrenheitToCelsius(float fahrenheit) -> float;
    auto convertCelsiusToKelvin(float celsius) -> float;
    auto convertFahrenheitToKelvin(float fahrenheit) -> float;

    constexpr float METERS_TO_FEET = 3.28084F;
    constexpr float FEET_TO_METERS = 1.0F / METERS_TO_FEET;
    constexpr float YARDS_TO_FEET = 3.0F;

    auto convertFeetToMeters(float feet) -> float;
    auto convertMetersToFeet(float meters) -> float;

    auto getDistanceInYards(Vector3D position) -> float;

    auto calcLandingPoint(const std::vector<Vector3D> &positions) -> Vector3D;
}

#endif // MATH_UTILS_HPP