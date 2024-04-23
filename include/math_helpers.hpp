#ifndef MATH_HELPERS_HPP
#define MATH_HELPERS_HPP

void calcRhoMetric(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcRhoImperial(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcc0(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcv0(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcv0x (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcv0y(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcv0z(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcwx(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcwy(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcwz(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcOmega(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcROmega(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcvxw(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcvyw(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcSVP(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcBarometricPressure(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcRe100(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);


typedef void (*CalcFuncPtr)(struct golfBall *ball, struct variables*, struct atmosphericData*);
// needs to be in a specific order, some variables depend on others
static CalcFuncPtr calcFuncs[] = {
    calcRhoMetric,
    calcRhoImperial,
    calcc0,
    calcv0,
    calcv0x,
    calcv0y,
    calcv0z,
    calcwx,
    calcwy,
    calcwz,
    calcOmega,
    calcROmega,
    calcvxw,
    calcvyw,
    calcSVP,
    calcBarometricPressure,
    calcRe100
};

void initVars (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);

#endif // MATH_HELPERS_HPP