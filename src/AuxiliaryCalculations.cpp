#include <math.h>

#include "AuxiliaryCalculations.hpp"
#include "GolfBallKinematics.hpp"
#include "math_constants.hpp"
#include "atmosphere.hpp"

void AuxiliaryCalculations::calculatePosition()
{
    // calculate balls 3d position
}

void AuxiliaryCalculations::calculateV()
{
    float vx = velocity3D[0] + acceleration3D[0] * GolfBallKinematics::dt;
}

void AuxiliaryCalculations::calculateVelocityw()
{
}

void AuxiliaryCalculations::calculateAccel()
{
    acceleration3D[0] = accelerationDrag3D[0] + accelertaionMagnitude3D[0];
    acceleration3D[1] = accelerationDrag3D[1] + accelertaionMagnitude3D[1];
    acceleration3D[2] = accelerationDrag3D[2] + accelertaionMagnitude3D[2] - 32.174;
}

void AuxiliaryCalculations::calculateAccelD()
{
    accelerationDrag3D[0] = -physicsVars.getC0() * determineCoefficientOfDrag() * velocity3D_w[0] * (velocity3D[0] - velocity3D_w[0]);
    accelerationDrag3D[1] = -physicsVars.getC0() * determineCoefficientOfDrag() * velocity3D_w[1] * (velocity3D[1] - velocity3D_w[1]);
    accelerationDrag3D[2] = -physicsVars.getC0() * determineCoefficientOfDrag() * velocity3D_w[2] * velocity3D[2];
}

void AuxiliaryCalculations::calculateAccelM()
{
    accelertaionMagnitude3D[0] = physicsVars.getC0() * (determineCoefficientOfLift() / physicsVars.getOmega()) * vw * (physicsVars.getW()[1] * velocity3D[2] - physicsVars.getW()[2] * (velocity3D[1] - velocity3D_w[1])) / w_perp_div_w;
    accelertaionMagnitude3D[1] = physicsVars.getC0() * (determineCoefficientOfLift() / physicsVars.getOmega()) * vw * (physicsVars.getW()[2] * (velocity3D[1] - physicsVars.getVw()[1]) - physicsVars.getW()[0] * velocity3D[2]) / w_perp_div_w;
    accelertaionMagnitude3D[2] = physicsVars.getC0() * (determineCoefficientOfLift() / physicsVars.getOmega()) * vw * (physicsVars.getW()[0] * (velocity3D[1] - physicsVars.getVw()[1]) - physicsVars.getW()[1] * (velocity3D[0] - velocity3D_w[0]) ) / w_perp_div_w;
}

void AuxiliaryCalculations::calculateTau()
{
}

void AuxiliaryCalculations::calculateRw()
{
}

void AuxiliaryCalculations::calculateVw()
{
}

void AuxiliaryCalculations::calculateRe_x_e5()
{
    // calculate Re x e^5
}

float AuxiliaryCalculations::determineCoefficientOfDrag()
{
    if (getRe_x_e5() <= math_constants::CdL)
    {
        return math_constants::CdL;
    }
    else if (getRe_x_e5() < 1)
    {
        return math_constants::CdL - (math_constants::CdL - math_constants::CdH) * (getRe_x_e5() - 0.5) / 0.5 + math_constants::CdS * getSpinFactor();
    }
    else
    {
        return math_constants::CdH + math_constants::CdS * getSpinFactor();
    }
}

float AuxiliaryCalculations::determineCoefficientOfLift()
{
    if (getSpinFactor() <= 0.3)
    {
        return math_constants::coeff1 * getSpinFactor() + math_constants::coeff2 * pow(getSpinFactor(), 2);
    }
    else
    {
        return 0.305;
    }
}

void AuxiliaryCalculations::calculateAllVariables()
{
    // call all calculate function
}