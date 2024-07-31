#include "test_helpers.hpp"

#include "atmosphere.hpp"
#include "golf_ball.hpp"

struct golfBall initBall(float a, float b, float c, float d, float e, float f,
                         float g, float h) {
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

struct atmosphericData initAtmos(float a, float b, float c, float d, float e,
                                 float f, float g) {
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
