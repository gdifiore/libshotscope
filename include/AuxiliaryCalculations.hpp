#ifndef AUXILIARYCALCULATIONS_HPP
#define AUXILIARYCALCULATIONS_HPP

#include "GolfBallPhysicsVariables.hpp"
#include "atmosphere.hpp"

class CoefficientModel;

class AuxiliaryCalculations
{
public:
    AuxiliaryCalculations(atmosphericData& atmos, GolfBallPhysicsVariables& vars, CoefficientModel& coeffModel);

    float getX() const { return m_x; };
    void calcX();

    float getVX() const { return m_vx; };
    void calcVX();

    float getAX() const { return m_ax; };
    void calcAX();

    float getADragX() const { return m_adragx; };
    void calcADragX();

    float getAMagX() const { return m_adragx; };
    void calcAMagX();

    float getVW() const { return m_vw; };
    void calcVW();

    float getVxW() const { return m_vxw; };
    void calcVxW();

    float getRe() const { return m_Re; };
    void calcRe();

    float getS() const { return m_S; };
    void calcS();

private:
    atmosphericData& atmos;
    GolfBallPhysicsVariables& vars;
    CoefficientModel& coeffModel;
    float m_x;
    float m_vx;
    float m_ax;
    float m_adragx;
    float m_aMagx;
    float m_vw;
    float m_vxw;

    float m_Re; // Reynolds Number (x10^-5)

    float m_S; // spin factor
};

#endif // AUXILIARYCALCULATIONS_HPP