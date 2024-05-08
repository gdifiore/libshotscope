#ifndef COEFFICIENTMODEL_HPP
#define COEFFICIENTMODEL_HPP

#include "math_constants.hpp"

class AuxiliaryCalculations;

class CoefficientModel
{
public:
    CoefficientModel(AuxiliaryCalculations& auxCalc);
    ~CoefficientModel();

    float determineCoefficientOfDrag();
    float determineCoefficientOfLift();

private:
    AuxiliaryCalculations& auxCalc;
};

#endif // COEFFICIENTMODEL_HPP