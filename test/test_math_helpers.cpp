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

    GolfBallPhysicsVariables ballVars(ball, atmos);

    ballVars.calculateAllVariables();
    EXPECT_NEAR(ballVars.getRhoImperial(), 0.0748, 0.001);
    EXPECT_NEAR(ballVars.getRhoMetric(), 1.194, 0.001);
    EXPECT_NEAR(ballVars.getC0(), 0.005682, 0.00001);
    EXPECT_NEAR(ballVars.getV0(), 234.72, 0.1);
    EXPECT_NEAR(ballVars.getV0x(), 0.0, 0.1);
    EXPECT_NEAR(ballVars.getV0y(), 230.41, 0.1);
    EXPECT_NEAR(ballVars.getV0z(), 44.79, 0.1);
    EXPECT_NEAR(ballVars.getWx(), 314.16, 0.1);
    EXPECT_NEAR(ballVars.getWy(), 0.0, 0.1);
    EXPECT_NEAR(ballVars.getWz(), 0.0, 0.1);
    EXPECT_NEAR(ballVars.getOmega(), 314.16, 0.1);
    EXPECT_NEAR(ballVars.getROmega(), 21.99, 0.01);
    EXPECT_NEAR(ballVars.getTempC(), 21.11, 0.1);
    EXPECT_NEAR(ballVars.getElevationM(), 0.0, 0.1);
    EXPECT_NEAR(ballVars.getVxw(), 0.0, 0.1);
    EXPECT_NEAR(ballVars.getVyw(), 0.0, 0.1);
    EXPECT_NEAR(ballVars.getSVP(), 18.79, 0.01);
    EXPECT_NEAR(ballVars.getBarometricPressure(), 759.97, 0.1);
    EXPECT_NEAR(ballVars.getRe100(), 123600, 100);
}


TEST(ShotScopeTest, initVarsNotDefault) {
    const struct golfBall ball = initBall(0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 500.0);
    const struct atmosphericData atmos = initAtmos(70.0, 90.0, 2.0, 30.0, 50.0, 50.0, 29.92);

    GolfBallPhysicsVariables vars(ball, atmos);

    vars.calculateAllVariables();
    EXPECT_NEAR(vars.getRhoImperial(), 0.0745, 0.001);
    EXPECT_NEAR(vars.getRhoMetric(), 1.190, 0.001);
    EXPECT_NEAR(vars.getC0(), 0.005663, 0.00001);
    EXPECT_NEAR(vars.getV0(), 234.72, 0.1);
    EXPECT_NEAR(vars.getV0x(), 0.0, 0.1);
    EXPECT_NEAR(vars.getV0y(), 230.41, 0.1);
    EXPECT_NEAR(vars.getV0z(), 44.79, 0.1);
    EXPECT_NEAR(vars.getWx(), 314.16, 0.1);
    EXPECT_NEAR(vars.getWy(), -9.99, 0.1);
    EXPECT_NEAR(vars.getWz(), 51.4, 0.1);
    EXPECT_NEAR(vars.getOmega(), 318.49, 0.1);
    EXPECT_NEAR(vars.getROmega(), 22.29, 0.01);
    EXPECT_NEAR(vars.getTempC(), 21.11, 0.1);
    EXPECT_NEAR(vars.getElevationM(), 27.4, 0.1);
    EXPECT_NEAR(vars.getVxw(), 1.5, 0.1);
    EXPECT_NEAR(vars.getVyw(), 2.5, 0.1);
    EXPECT_NEAR(vars.getSVP(), 18.79, 0.01);
    EXPECT_NEAR(vars.getBarometricPressure(), 759.97, 0.1);
    EXPECT_NEAR(vars.getRe100(), 123200, 100);
}