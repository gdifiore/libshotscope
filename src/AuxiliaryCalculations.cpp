#include "AuxiliaryCalculations.hpp"
#include "GolfBallKinematics.hpp"
#include "CoefficientModel.hpp"

AuxiliaryCalculations::AuxiliaryCalculations(GolfBallPhysicsVariables vars, CoefficientModel& coeffModel) : vars(vars), coeffModel(coeffModel)
{
}

void AuxiliaryCalculations::calcX()
{
    m_x = m_x + m_vx * GolfBallKinematics::dt + 0.5 * m_ax * GolfBallKinematics::dt * GolfBallKinematics::dt;
}

void AuxiliaryCalculations::calcVX()
{
    m_vx = m_vx + m_ax * GolfBallKinematics::dt;
}

void AuxiliaryCalculations::calcAX()
{
    m_ax = m_adragx + m_aMagx;
}

void AuxiliaryCalculations::calcADragX()
{
    m_adragx = -vars.getC0() * coeffModel.determineCoefficientOfDrag() * m_vw * (m_vx - m_vxw);
}

void AuxiliaryCalculations::calcRe()
{

}