#ifndef COEFFICIENTMODEL_HPP
#define COEFFICIENTMODEL_HPP

#include "AuxIliaryCalculations.hpp"
#include "math_constants.hpp"

class CoefficientModel
{
public:
    CoefficientModel(AuxiliaryCalculations auxCalc);
    ~CoefficientModel();

    float determineCoefficientOfDrag();
    float determineCoefficientOfLift();

private:
    AuxiliaryCalculations auxCalc;
};

#endif // COEFFICIENTMODEL_HPP