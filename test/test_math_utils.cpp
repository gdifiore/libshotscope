#include <gtest/gtest.h>

#include <libshotscope.hpp>

TEST(ShotScopeTest, FtoC)
{
    float tempC = math_utils::convertFahrenheitToCelsius(32.0);

    EXPECT_NEAR(tempC, 0.0, 0.1);
}

TEST(ShotScopeTest, CtoK)
{
    float tempK = math_utils::convertCelsiusToKelvin(32.0);

    EXPECT_NEAR(tempK, 305.15, 0.1);
}

TEST(ShotScopeTest, FtoK)
{
    float tempK = math_utils::convertFahrenheitToKelvin(32.0);

    EXPECT_NEAR(tempK, 273.16, 0.1);
}

TEST(ShotScopeTest, FTtoM)
{
    float tempK = math_utils::convertFeetToMeters(5.0);

    EXPECT_NEAR(tempK, 1.524, 0.1);
}

TEST(ShotScopeTest, MtoFT)
{
    float lengthFT = math_utils::convertMetersToFeet(10.0);

    EXPECT_NEAR(lengthFT, 32.8084, 0.01);
}

TEST(ShotScopeTest, DistanceInYards)
{
    Vector3D position{3.0, 4.0, 0.0};
    float distanceYards = math_utils::getDistanceInYards(position);

    EXPECT_NEAR(distanceYards, 1.66, 0.01);
}

// Vector math tests
TEST(ShotScopeTest, DotProductOrthogonalVectors)
{
    Vector3D a{1.0, 0.0, 0.0};
    Vector3D b{0.0, 1.0, 0.0};
    float result = math_utils::dotProduct(a, b);

    EXPECT_NEAR(result, 0.0, 0.001);
}

TEST(ShotScopeTest, DotProductParallelVectors)
{
    Vector3D a{3.0, 0.0, 0.0};
    Vector3D b{2.0, 0.0, 0.0};
    float result = math_utils::dotProduct(a, b);

    EXPECT_NEAR(result, 6.0, 0.001);
}

TEST(ShotScopeTest, DotProductGeneralCase)
{
    Vector3D a{1.0, 2.0, 3.0};
    Vector3D b{4.0, 5.0, 6.0};
    float result = math_utils::dotProduct(a, b);

    // 1*4 + 2*5 + 3*6 = 4 + 10 + 18 = 32
    EXPECT_NEAR(result, 32.0, 0.001);
}

TEST(ShotScopeTest, CrossProductOrthogonalVectors)
{
    Vector3D a{1.0, 0.0, 0.0};
    Vector3D b{0.0, 1.0, 0.0};
    Vector3D result = math_utils::crossProduct(a, b);

    // i x j = k
    EXPECT_NEAR(result[0], 0.0, 0.001);
    EXPECT_NEAR(result[1], 0.0, 0.001);
    EXPECT_NEAR(result[2], 1.0, 0.001);
}

TEST(ShotScopeTest, CrossProductParallelVectors)
{
    Vector3D a{1.0, 2.0, 3.0};
    Vector3D b{2.0, 4.0, 6.0};
    Vector3D result = math_utils::crossProduct(a, b);

    // Parallel vectors have zero cross product
    EXPECT_NEAR(result[0], 0.0, 0.001);
    EXPECT_NEAR(result[1], 0.0, 0.001);
    EXPECT_NEAR(result[2], 0.0, 0.001);
}

TEST(ShotScopeTest, CrossProductGeneralCase)
{
    Vector3D a{1.0, 2.0, 3.0};
    Vector3D b{4.0, 5.0, 6.0};
    Vector3D result = math_utils::crossProduct(a, b);

    // (2*6 - 3*5, 3*4 - 1*6, 1*5 - 2*4) = (-3, 6, -3)
    EXPECT_NEAR(result[0], -3.0, 0.001);
    EXPECT_NEAR(result[1], 6.0, 0.001);
    EXPECT_NEAR(result[2], -3.0, 0.001);
}

TEST(ShotScopeTest, ReflectVerticalBounce)
{
    Vector3D velocity{0.0, 0.0, -10.0};  // Moving downward
    Vector3D normal{0.0, 0.0, 1.0};      // Ground normal (pointing up)
    Vector3D result = math_utils::reflect(velocity, normal);

    // Should reflect upward
    EXPECT_NEAR(result[0], 0.0, 0.001);
    EXPECT_NEAR(result[1], 0.0, 0.001);
    EXPECT_NEAR(result[2], 10.0, 0.001);
}

TEST(ShotScopeTest, ReflectAngledBounce)
{
    Vector3D velocity{5.0, 0.0, -5.0};  // Moving at 45 degrees down
    Vector3D normal{0.0, 0.0, 1.0};     // Ground normal
    Vector3D result = math_utils::reflect(velocity, normal);

    // Horizontal component unchanged, vertical reversed
    EXPECT_NEAR(result[0], 5.0, 0.001);
    EXPECT_NEAR(result[1], 0.0, 0.001);
    EXPECT_NEAR(result[2], 5.0, 0.001);
}

TEST(ShotScopeTest, ReflectWallBounce)
{
    Vector3D velocity{10.0, 5.0, 0.0};  // Moving toward wall
    Vector3D normal{-1.0, 0.0, 0.0};    // Wall normal (pointing left)
    Vector3D result = math_utils::reflect(velocity, normal);

    // X component reversed, others unchanged
    EXPECT_NEAR(result[0], -10.0, 0.001);
    EXPECT_NEAR(result[1], 5.0, 0.001);
    EXPECT_NEAR(result[2], 0.0, 0.001);
}

TEST(ShotScopeTest, ProjectOntoAxisAligned)
{
    Vector3D vector{3.0, 4.0, 0.0};
    Vector3D onto{1.0, 0.0, 0.0};  // X-axis
    Vector3D result = math_utils::project(vector, onto);

    // Projection onto x-axis should be (3, 0, 0)
    EXPECT_NEAR(result[0], 3.0, 0.001);
    EXPECT_NEAR(result[1], 0.0, 0.001);
    EXPECT_NEAR(result[2], 0.0, 0.001);
}

TEST(ShotScopeTest, ProjectParallelVectors)
{
    Vector3D vector{3.0, 6.0, 9.0};
    Vector3D onto{1.0, 2.0, 3.0};
    Vector3D result = math_utils::project(vector, onto);

    // Parallel vectors, projection should equal the original vector
    EXPECT_NEAR(result[0], 3.0, 0.001);
    EXPECT_NEAR(result[1], 6.0, 0.001);
    EXPECT_NEAR(result[2], 9.0, 0.001);
}

TEST(ShotScopeTest, ProjectOrthogonalVectors)
{
    Vector3D vector{1.0, 0.0, 0.0};
    Vector3D onto{0.0, 1.0, 0.0};
    Vector3D result = math_utils::project(vector, onto);

    // Orthogonal vectors, projection should be zero
    EXPECT_NEAR(result[0], 0.0, 0.001);
    EXPECT_NEAR(result[1], 0.0, 0.001);
    EXPECT_NEAR(result[2], 0.0, 0.001);
}

TEST(ShotScopeTest, ProjectGeneralCase)
{
    Vector3D vector{1.0, 2.0, 3.0};
    Vector3D onto{2.0, 1.0, 0.0};
    Vector3D result = math_utils::project(vector, onto);

    // dot(v, onto) = 1*2 + 2*1 + 3*0 = 4
    // dot(onto, onto) = 2*2 + 1*1 + 0*0 = 5
    // projection = (4/5) * onto = (8/5, 4/5, 0)
    EXPECT_NEAR(result[0], 1.6, 0.001);
    EXPECT_NEAR(result[1], 0.8, 0.001);
    EXPECT_NEAR(result[2], 0.0, 0.001);
}
