#include <cmath>

#include "CoefficientModel.hpp"
#include "AuxiliaryCalculations.hpp"
#include "math_constants.hpp"

CoefficientModel::CoefficientModel(AuxiliaryCalculations& auxCalc) : auxCalc(auxCalc)
{
}

float CoefficientModel::determineCoefficientOfDrag()
{
    if (auxCalc.getRe() <= math_constants::CdL)
    {
        return math_constants::CdL;
    }
    else if (auxCalc.getRe() < 1)
    {
        return math_constants::CdL - (math_constants::CdL - math_constants::CdH) * (auxCalc.getRe() - 0.5) / 0.5 + math_constants::CdS * auxCalc.getS();
    }
    else
    {
        return math_constants::CdH + math_constants::CdS * auxCalc.getS();
    }
}

float CoefficientModel::determineCoefficientOfLift()
{
    if (auxCalc.getS() <= 0.3)
    {
        return math_constants::coeff1 * auxCalc.getS() + math_constants::coeff2 * pow(auxCalc.getS(), 2);
    }
    else
    {
        return 0.305;
    }
}
