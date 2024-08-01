#include <gtest/gtest.h>

#include "math_utils.hpp"

TEST(ShotScopeTest, FtoC) {
    float tempC = math_utils::convertFahrenheitToCelsius(32.0);

    EXPECT_NEAR(tempC, 0.0, 0.1);
}

TEST(ShotScopeTest, CtoK) {
    float tempK = math_utils::convertCelsiusToKelvin(32.0);

    EXPECT_NEAR(tempK, 305.15, 0.1);
}

TEST(ShotScopeTest, FtoK) {
    float tempK = math_utils::convertFahrenheitToKelvin(32.0);

    EXPECT_NEAR(tempK, 273.16, 0.1);
}

TEST(ShotScopeTest, FTtoM) {
    float tempK = math_utils::convertFeetToMeters(5.0);

    EXPECT_NEAR(tempK, 1.524, 0.1);
}

TEST(ShotScopeTest, MtoFT) {
    float lengthFT = math_utils::convertMetersToFeet(10.0);

    EXPECT_NEAR(lengthFT, 32.8084, 0.01);
}

TEST(ShotScopeTest, DistanceInYards) {
    Vector3D position {3.0, 4.0, 0.0};
    float distanceYards = math_utils::getDistanceInYards(position);

    EXPECT_NEAR(distanceYards, 1.66, 0.01);
}
