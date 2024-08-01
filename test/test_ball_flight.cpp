#include <gtest/gtest.h>

#include "GolfBallFlight.hpp"
#include "GolfBallPhysicsVariables.hpp"
#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "math_constants.hpp"
#include "math_utils.hpp"

TEST(ShotScopeTest, intermediaryCalc) {
  const golfBall ball {0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 500.0};
  const atmosphericData atmos {70.0, 90.0, 2.0, 30.0, 50.0, 50.0, 29.92};

  auto vars = GolfBallPhysicsVariables(ball, atmos);
  vars.calculateAllVariables();
  auto intermediary = GolfBallFlight(vars, ball, atmos);
  // just testing the initial line in the excel sheet w/ initialization
  //intermediary.calculateAllVariables();

  EXPECT_NEAR(intermediary.getPhi(), 0.0, 0.1);
  EXPECT_NEAR(intermediary.getVelocity3D()[0], 0.0, 0.1);
  EXPECT_NEAR(intermediary.getVelocity3D()[1], 230.41, 0.1);
  EXPECT_NEAR(intermediary.getVelocity3D()[2], 44.79, 0.1);
  EXPECT_NEAR(intermediary.getV(), 234.7, 0.1);
  EXPECT_NEAR(intermediary.getVMph(), 160.0, 0.1);
  EXPECT_NEAR(intermediary.getTau(), 14.909, 0.1);
  EXPECT_NEAR(intermediary.getRw(), 22.29, 0.1);
  EXPECT_NEAR(intermediary.getWPerp(), 0.0, 0.1);
  EXPECT_NEAR(intermediary.getVw(), 234.7, 0.1);
  EXPECT_NEAR(intermediary.getVwMph(), 160.0, 0.1);
  EXPECT_NEAR(intermediary.getRe_x_e5(), 1.972, 0.1);
  EXPECT_NEAR(intermediary.determineCoefficientOfDrag(), 0.217, 0.1);
  EXPECT_NEAR(intermediary.getSpinFactor(), 0.0950, 0.1);
  EXPECT_NEAR(intermediary.determineCoefficientOfLift(), 0.1597, 0.1);
  EXPECT_NEAR(intermediary.getVelocity3D_w()[0], 0.0, 0.1);
  EXPECT_NEAR(intermediary.getVelocity3D_w()[1], 0.0, 0.1); // only x/y components
  EXPECT_NEAR(intermediary.getAccelerationDrag3D()[0], 0.0, 0.1);
  EXPECT_NEAR(intermediary.getAccelerationDrag3D()[1], -66.493, 0.1);
  EXPECT_NEAR(intermediary.getAccelerationDrag3D()[2], -12.925, 0.1);
  EXPECT_NEAR(intermediary.getWPerpDivW(), 1.0, 0.1);
  EXPECT_NEAR(intermediary.getAccelertaionMagnitude3D()[0], -8.191, 0.1);
  EXPECT_NEAR(intermediary.getAccelertaionMagnitude3D()[1], -9.377, 0.1);
  EXPECT_NEAR(intermediary.getAccelertaionMagnitude3D()[2], 48.241, 0.1);
  EXPECT_NEAR(intermediary.getAcceleration3D()[0], -8.191, 0.1);
  EXPECT_NEAR(intermediary.getAcceleration3D()[1], -75.87, 0.1);
  EXPECT_NEAR(intermediary.getAcceleration3D()[2], 3.142, 0.1);
  EXPECT_NEAR(intermediary.getPosition()[0], 0.0, 0.1);
  EXPECT_NEAR(intermediary.getPosition()[1], 0.0, 0.1);
  EXPECT_NEAR(intermediary.getPosition()[2], 0.0, 0.1);
}