#ifndef MATH_HELPERS_HPP
#define MATH_HELPERS_HPP

float convertFahrenheitToCelsius(float fahrenheit);
float convertCelsiusToKelvin(float celsius);
float convertFarenheitToKelvin(float fahrenheit);

void calcRhoMetric(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcRhoImperial(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcc0(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcv0(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcv0x (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);
void calcRe100(struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);


typedef void (*CalcFuncPtr)(struct golfBall *ball, struct variables*, struct atmosphericData*);
// needs to be in a specific order, some variables depend on others
static CalcFuncPtr calcFuncs[] = {
    calcRhoMetric,
    calcRhoImperial,
    calcc0,
    calcv0,
    calcv0x,
    calcRe100
};

void initVars (struct golfBall *ball, struct variables *vars, struct atmosphericData *atmos);

#endif // MATH_HELPERS_HPP