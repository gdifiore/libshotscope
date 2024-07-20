#include <gtest/gtest.h>

#include "atmosphere.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "golf_ball.hpp"
#include "math_constants.hpp"
#include "math_utils.hpp"

struct golfBall initBall(float a, float b, float c, float d, float e, float f, float g, float h) {
    struct golfBall testBall;

    testBall.x0 = a;
    testBall.y0 = b;
    testBall.z0 = c;
    testBall.exitSpeed = d;
    testBall.launchAngle = e;
    testBall.direction = f;
    testBall.backspin = g;
    testBall.sidespin = h;

    return testBall;
}

struct atmosphericData initAtmos(float a, float b, float c, float d, float e, float f, float g) {
    struct atmosphericData atmosData;

    atmosData.temp = a;
    atmosData.elevation = b;
    atmosData.vWind = c;
    atmosData.phiWind = d;
    atmosData.hWind = e;
    atmosData.relHumidity = f;
    atmosData.pressure = g;

    return atmosData;
}

// Test default initial values from a spreadsheet created by Alan M. Nathan at U. of Illinois
TEST(ShotScopeTest, initVarsDefault) {
    const struct golfBall ball = initBall(0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0);
    const struct atmosphericData atmos = initAtmos(70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92);

    auto ballVars = GolfBallPhysicsVariables::Builder()
                        .setGolfBall(ball)
                        .setAtmosphere(atmos)
                        .build();

    ballVars.calculateAllVariables();
    EXPECT_NEAR(ballVars.getRhoImperial(), 0.0748, 0.001);
    EXPECT_NEAR(ballVars.getRhoMetric(), 1.194, 0.001);
    EXPECT_NEAR(ballVars.getC0(), 0.005682, 0.00001);
    EXPECT_NEAR(ballVars.getV0(), 234.72, 0.1);
    EXPECT_NEAR(ballVars.getV0Vector()[0], 0.0, 0.1);  // V0x
    EXPECT_NEAR(ballVars.getV0Vector()[1], 230.41, 0.1);  // V0y
    EXPECT_NEAR(ballVars.getV0Vector()[2], 44.79, 0.1);  // V0z
    EXPECT_NEAR(ballVars.getW()[0], 314.16, 0.1);  // Wx
    EXPECT_NEAR(ballVars.getW()[1], 0.0, 0.1);  // Wy
    EXPECT_NEAR(ballVars.getW()[2], 0.0, 0.1);  // Wz
    EXPECT_NEAR(ballVars.getOmega(), 314.16, 0.1);
    EXPECT_NEAR(ballVars.getROmega(), 21.99, 0.01);
    EXPECT_NEAR(ballVars.getTempC(), 21.11, 0.1);
    EXPECT_NEAR(ballVars.getElevationM(), 0.0, 0.1);
    EXPECT_NEAR(ballVars.getVw()[0], 0.0, 0.1);  // Vxw
    EXPECT_NEAR(ballVars.getVw()[1], 0.0, 0.1);  // Vyw
    EXPECT_NEAR(ballVars.getSVP(), 18.79, 0.01);
    EXPECT_NEAR(ballVars.getBarometricPressure(), 759.97, 0.1);
    EXPECT_NEAR(ballVars.getRe100(), 123600, 100);
}

TEST(ShotScopeTest, initVarsNotDefault) {
    const struct golfBall ball = initBall(0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 500.0);
    const struct atmosphericData atmos = initAtmos(70.0, 90.0, 2.0, 30.0, 50.0, 50.0, 29.92);

    auto vars = GolfBallPhysicsVariables::Builder()
                    .setGolfBall(ball)
                    .setAtmosphere(atmos)
                    .build();

    vars.calculateAllVariables();
    EXPECT_NEAR(vars.getRhoImperial(), 0.0745, 0.001);
    EXPECT_NEAR(vars.getRhoMetric(), 1.190, 0.001);
    EXPECT_NEAR(vars.getC0(), 0.005663, 0.00001);
    EXPECT_NEAR(vars.getV0(), 234.72, 0.1);
    EXPECT_NEAR(vars.getV0Vector()[0], 0.0, 0.1);  // V0x
    EXPECT_NEAR(vars.getV0Vector()[1], 230.41, 0.1);  // V0y
    EXPECT_NEAR(vars.getV0Vector()[2], 44.79, 0.1);  // V0z
    EXPECT_NEAR(vars.getW()[0], 314.16, 0.1);  // Wx
    EXPECT_NEAR(vars.getW()[1], -9.99, 0.1);  // Wy
    EXPECT_NEAR(vars.getW()[2], 51.4, 0.1);  // Wz
    EXPECT_NEAR(vars.getOmega(), 318.49, 0.1);
    EXPECT_NEAR(vars.getROmega(), 22.29, 0.01);
    EXPECT_NEAR(vars.getTempC(), 21.11, 0.1);
    EXPECT_NEAR(vars.getElevationM(), 27.4, 0.1);
    EXPECT_NEAR(vars.getVw()[0], 1.5, 0.1);  // Vxw
    EXPECT_NEAR(vars.getVw()[1], 2.5, 0.1);  // Vyw
    EXPECT_NEAR(vars.getSVP(), 18.79, 0.01);
    EXPECT_NEAR(vars.getBarometricPressure(), 759.97, 0.1);
    EXPECT_NEAR(vars.getRe100(), 123200, 100);
}