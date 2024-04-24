#include <gtest/gtest.h>

#include "atmosphere.hpp"
#include "golf_ball.hpp"
#include "math_constants.hpp"
#include "math_helpers.hpp"
#include "math_utils.hpp"
#include "math_variables.hpp"

struct golfBall* initBall() {
    // Dynamically allocate memory for a golfBall instance
    struct golfBall* testBall = (struct golfBall*)malloc(sizeof(struct golfBall));

    // Check if memory allocation was successful
    if (testBall == NULL) {
        printf("Error: Memory allocation failed.\n");
        return NULL;
    }

    // Initialize the members of the dynamically allocated golfBall
    testBall->x0 = 0.0f;
    testBall->y0 = 0.0f;
    testBall->z0 = 0.0f;
    testBall->exitSpeed = 160.0f;
    testBall->launchAngle = 11.0f;
    testBall->direction = 0.0f;
    testBall->backspin = 3000.0f;
    testBall->sidespin = 0.0f;

    return testBall;
}

struct atmosphericData* initAtmos() {
    // Dynamically allocate memory for an atmosphericData instance
    struct atmosphericData* atmosData = (struct atmosphericData*)malloc(sizeof(struct atmosphericData));

    // Check if memory allocation was successful
    if (atmosData == NULL) {
        printf("Error: Memory allocation failed.\n");
        return NULL;
    }

    // Initialize the members of the dynamically allocated atmosphericData
    atmosData->temp = 70.0f;
    atmosData->elevation = 0.0f;
    atmosData->vWind = 0.0f;
    atmosData->phiWind = 0.0f;
    atmosData->hWind = 0.0f;
    atmosData->relHumidity = 50.0f;
    atmosData->pressure = 29.92f;

    return atmosData;
}

// Demonstrate some basic assertions.
TEST(ShotScopeTest, initVars) {
    struct golfBall *ball = initBall();
    struct atmosphericData *atmos = initAtmos();
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