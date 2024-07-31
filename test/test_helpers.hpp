#ifndef TEST_HELPERS_HPP
#define TEST_HELPERS_HPP

#include "atmosphere.hpp"
#include "golf_ball.hpp"

struct golfBall initBall(float a, float b, float c, float d, float e, float f,
                         float g, float h);

struct atmosphericData initAtmos(float a, float b, float c, float d, float e,
                                 float f, float g);

#endif  // TEST_HELPERS_HPP