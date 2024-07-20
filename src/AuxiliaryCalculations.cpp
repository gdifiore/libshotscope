#include <math.h>

#include "AuxiliaryCalculations.hpp"
#include "GolfBallKinematics.hpp"
#include "CoefficientModel.hpp"
#include "atmosphere.hpp"

AuxiliaryCalculations::AuxiliaryCalculations(atmosphericData& atmos, GolfBallPhysicsVariables& vars, CoefficientModel& coeffModel) : atmos(atmos), vars(vars), coeffModel(coeffModel)
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

void AuxiliaryCalculations::calcVW()
{
    double z = vars.getZ();

    if (vars. >= atmos.hWind) {
        m_vw = sqrt(pow((m_vx - m_vxw), 2) + pow((m_vy - m_vyw), 2) + pow(m_vz, 2));
    } else {
        m_vw = m_v;
    }
}

void AuxiliaryCalculations::calcVxW()
{
    m_vxw = 0.0;
}

void AuxiliaryCalculations::calcRe()
{

}