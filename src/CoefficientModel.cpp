#include <cmath>

#include "CoefficientModel.hpp"
#include "math_constants.hpp"

CoefficientModel::CoefficientModel(AuxiliaryCalculations auxCalc) {
    auxCalc = auxCalc;
}

CoefficientModel::~CoefficientModel() {
    // Destructor implementation
}

float CoefficientModel::determineCoefficientOfDrag() {
    if (auxCalc.Re <= math_constants::CdL) {
        return math_constants::CdL;
    } else if (auxCalc.Re < 1) {
        return math_constants::CdL - (math_constants::CdL - math_constants::CdH) * (auxCalc.Re - 0.5) / 0.5 + math_constants::CdS * auxCalc.S;
    } else {
        return math_constants::CdH + math_constants::CdS * auxCalc.S;
    }
}

float CoefficientModel::determineCoefficientOfLift() {
    if (auxCalc.S <= 0.3) {
        return math_constants::coeff1 * auxCalc.S + math_constants::coeff2 * pow(auxCalc.S, 2);
    } else {
        return 0.305;
    }
}
