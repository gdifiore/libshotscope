#include <gtest/gtest.h>

#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "math_constants.hpp"
#include "math_helpers.hpp"
#include "math_utils.hpp"
#include "math_variables.hpp"

struct golfBall* initBall(float a, float b, float c, float d, float e, float f, float g, float h) {
    // Dynamically allocate memory for a golfBall instance
    struct golfBall* testBall = (struct golfBall*)malloc(sizeof(struct golfBall));

    // Check if memory allocation was successful
    if (testBall == NULL) {
        printf("Error: Memory allocation failed.\n");
        return NULL;
    }

    // Initialize the members of the dynamically allocated golfBall
    testBall->x0 = a;
    testBall->y0 = b;
    testBall->z0 = c;
    testBall->exitSpeed = d;
    testBall->launchAngle = e;
    testBall->direction = f;
    testBall->backspin = g;
    testBall->sidespin = h;

    return testBall;
}

struct atmosphericData* initAtmos(float a, float b, float c, float d, float e, float f, float g) {
    // Dynamically allocate memory for an atmosphericData instance
    struct atmosphericData* atmosData = (struct atmosphericData*)malloc(sizeof(struct atmosphericData));

    // Check if memory allocation was successful
    if (atmosData == NULL) {
        printf("Error: Memory allocation failed.\n");
        return NULL;
    }

    // Initialize the members of the dynamically allocated atmosphericData
    atmosData->temp = a;
    atmosData->elevation = b;
    atmosData->vWind = c;
    atmosData->phiWind = d;
    atmosData->hWind = e;
    atmosData->relHumidity = f;
    atmosData->pressure = g;

    return atmosData;
}

// Test default initial values from a spreadsheet created by Alan M. Nathan at U. of Illinois
TEST(ShotScopeTest, initVarsDefault) {
    struct golfBall *ball = initBall(0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 0.0);
    struct atmosphericData *atmos = initAtmos(70.0, 0.0, 0.0, 0.0, 0.0, 50.0, 29.92);
    struct variables *vars = (struct variables*)malloc(sizeof(struct variables));

    initVars(ball, vars, atmos);
    EXPECT_NEAR(vars->rhoImperial, 0.0748, 0.001);
    EXPECT_NEAR(vars->rhoMetric, 1.194, 0.001);
    EXPECT_NEAR(vars->c0, 0.005682, 0.000001);
    EXPECT_NEAR(vars->v0, 234.72, 0.1);
    EXPECT_NEAR(vars->v0x, 0.0, 0.1);
    EXPECT_NEAR(vars->v0y, 230.41, 0.1);
    EXPECT_NEAR(vars->v0z, 44.79, 0.1);
    EXPECT_NEAR(vars->wx, 314.16, 0.1);
    EXPECT_NEAR(vars->wy, 0.0, 0.1);
    EXPECT_NEAR(vars->wz, 0.0, 0.1);
    EXPECT_NEAR(vars->omega, 314.16, 0.1);
    EXPECT_NEAR(vars->rOmega, 21.99, 0.01);
    EXPECT_NEAR(vars->tempC, 21.11, 0.1);
    EXPECT_NEAR(vars->elevationM, 0.0, 0.1);
    EXPECT_NEAR(vars->vxw, 0.0, 0.1);
    EXPECT_NEAR(vars->vyw, 0.0, 0.1);
    EXPECT_NEAR(vars->SVP, 18.79, 0.01);
    EXPECT_NEAR(vars->barometricPressure, 759.97, 0.1);
    EXPECT_NEAR(vars->Re100, 123600, 100);

    free(ball);
    free(atmos);
    free(vars);
}

TEST(ShotScopeTest, initVarsNotDefault) {
    struct golfBall *ball = initBall(0.0, 0.0, 0.0, 160.0, 11.0, 0.0, 3000.0, 500.0);
    struct atmosphericData *atmos = initAtmos(70.0, 90.0, 2.0, 30.0, 50.0, 50.0, 29.92);
    struct variables *vars = (struct variables*)malloc(sizeof(struct variables));

    initVars(ball, vars, atmos);
    EXPECT_NEAR(vars->rhoImperial, 0.0745, 0.001);
    EXPECT_NEAR(vars->rhoMetric, 1.190, 0.001);
    EXPECT_NEAR(vars->c0, 0.005663, 0.000001);
    EXPECT_NEAR(vars->v0, 234.72, 0.1);
    EXPECT_NEAR(vars->v0x, 0.0, 0.1);
    EXPECT_NEAR(vars->v0y, 230.41, 0.1);
    EXPECT_NEAR(vars->v0z, 44.79, 0.1);
    EXPECT_NEAR(vars->wx, 314.16, 0.1);
    EXPECT_NEAR(vars->wy, -9.99, 0.1);
    EXPECT_NEAR(vars->wz, 51.4, 0.1);
    EXPECT_NEAR(vars->omega, 318.49, 0.1);
    EXPECT_NEAR(vars->rOmega, 22.29, 0.01);
    EXPECT_NEAR(vars->tempC, 21.11, 0.1);
    EXPECT_NEAR(vars->elevationM, 27.4, 0.1);
    EXPECT_NEAR(vars->vxw, 1.5, 0.1);
    EXPECT_NEAR(vars->vyw, 2.5, 0.1);
    EXPECT_NEAR(vars->SVP, 18.79, 0.01);
    EXPECT_NEAR(vars->barometricPressure, 759.97, 0.1);
    EXPECT_NEAR(vars->Re100, 123200, 100);

    free(ball);
    free(atmos);
    free(vars);
}