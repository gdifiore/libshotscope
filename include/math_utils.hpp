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

    auto convertFeetToMeters(float feet) -> float;
    auto convertMetersToFeet(float meters) -> float;

    auto getDistanceInYards(Vector3D position) -> float;

    // Vector math operations
    auto dot(const Vector3D& a, const Vector3D& b) noexcept -> float;
    auto cross(const Vector3D& a, const Vector3D& b) noexcept -> Vector3D;
    auto magnitude(const Vector3D& v) noexcept -> float;
    auto normalize(const Vector3D& v) -> Vector3D;
    auto project(const Vector3D& v, const Vector3D& onto) -> Vector3D;
}

#endif // MATH_UTILS_HPP