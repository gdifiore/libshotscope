#ifndef AUXILIARYCALCULATIONS_HPP
#define AUXILIARYCALCULATIONS_HPP

#include <array>
#include <optional>
#include <stdexcept>

#include "GolfBallPhysicsVariables.hpp"

class AuxiliaryCalculations
{
public:
    using Vector3D = std::array<float, 3>;

    // Builder class for easier construction
    class Builder;

    // Deleted copy constructor and assignment operator
    AuxiliaryCalculations(const AuxiliaryCalculations &) = delete;
    AuxiliaryCalculations &operator=(const AuxiliaryCalculations &) = delete;

    // Move constructor and assignment operator
    AuxiliaryCalculations(AuxiliaryCalculations &&) noexcept = default;
    AuxiliaryCalculations &operator=(AuxiliaryCalculations &&) noexcept = default;

    // Getters
    float getV() const { return v; }
    float getVMph() const { return vMph; }
    const Vector3D& getVelocity3D() const { return velocity3D; }
    float getTau() const { return tau; }
    float getRw() const { return rw; }
    float getWPerp() const { return w_perp; }
    float getWPerpDivW() const { return w_perp_div_w; }
    float getRe_x_e5() const { return Re_x_e5; }
    float getVw() const { return vw; }
    float getVwMph() const { return vwMph; }
    float getSpinFactor() const { return spinFactor; }
    const Vector3D& getVelocity3D_w() const { return velocity3D_w; }
    const Vector3D& getAccelerationDrag3D() const { return accelerationDrag3D; }
    const Vector3D& getAccelertaionMagnitude3D() const { return accelertaionMagnitude3D; }
    const Vector3D& getAcceleration3D() const { return acceleration3D; }

    // Want below to be in its own Kinematics Class
    const Vector3D& getPosition() const { return position; }

    // Want below to be in its own CoefficientModel class
    float determineCoefficientOfDrag();
    float determineCoefficientOfLift();

    // Calculate all variables
    void calculateAllVariables();

private:
    // Private constructor, use Builder to create instances
    AuxiliaryCalculations(GolfBallPhysicsVariables &physicsVars/*, CoefficientModel &coeffModel*/);

    GolfBallPhysicsVariables &physicsVars;
    //CoefficientModel &coeffModel;

    // kinematics class
    Vector3D position;

    // Member variables
    float v;
    float vMph;
    Vector3D velocity3D;

    float tau;
    float rw;
    float w_perp = 0.0f;
    float w_perp_div_w = 1.0f;
    float Re_x_e5;
    float vw;
    float vwMph;
    float spinFactor; // S
    Vector3D velocity3D_w;
    Vector3D accelerationDrag3D;
    Vector3D accelertaionMagnitude3D;
    Vector3D acceleration3D;

    // Private calculation methods
    void calculateV();
    void calculateVelocityw();

    void calculateAccel();
    void calculateAccelD();
    void calculateAccelM();

    void calculateTau();
    void calculateRw();
    void calculateVw();
    void calculateRe_x_e5();

    // Want below to be in its own Kinematics Class
    void calculatePosition();
};

// Builder class definition

class AuxiliaryCalculations::Builder

{

public:
    Builder &setPhysicsVars(GolfBallPhysicsVariables &physicsVars)
    {
        this->physicsVars = &physicsVars;

        return *this;
    }

    /*Builder &setCoefficientModel(CoefficientModel &coeffModel)
    {
        this->coeffModel = &coeffModel;

        return *this;
    }*/

    AuxiliaryCalculations build() const
    {
        if (!physicsVars /*!|| coeffModel*/)
        {
            throw std::runtime_error("GolfBallPhysicsVariables must be set!");
            //throw std::runtime_error("GolfBallPhysicsVariables and CoefficientModel must be set!");
        }
        return AuxiliaryCalculations(*physicsVars/*, *coeffModel*/);
    }

private:
    GolfBallPhysicsVariables *physicsVars = nullptr;
    //CoefficientModel *coeffModel = nullptr;
};

#endif // AUXILIARYCALCULATIONS_HPP