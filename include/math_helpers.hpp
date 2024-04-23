#ifndef MATH_HELPERS_HPP
#define MATH_HELPERS_HPP

float convertFahrenheitToCelsius(float fahrenheit);
float convertCelsiusToKelvin(float celsius);
float convertFarenheitToKelvin(float fahrenheit);

void calcRhoMetric(struct variables *vars, struct atmosphericData *atmos);
void calcRe100(struct variables *vars, struct atmosphericData *atmos);

typedef void (*CalcFuncPtr)(struct variables*, struct atmosphericData*);
// needs to be in a specific order, some variables depend on others
static CalcFuncPtr calcFuncs[] = {
    calcRhoMetric,
    calcRe100
};


#endif // MATH_HELPERS_HPP