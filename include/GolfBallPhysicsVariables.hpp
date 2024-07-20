#ifndef GOLFBALLPHYSICSVARIABLES_HPP
#define GOLFBALLPHYSICSVARIABLES_HPP

#include <array>
#include <optional>
#include "golf_ball.hpp"
#include "atmosphere.hpp"
#include <stdexcept>

class GolfBallPhysicsVariables
{
public:
    using Vector3D = std::array<float, 3>;

    // Builder class for easier construction
    class Builder;

    // Deleted copy constructor and assignment operator
    GolfBallPhysicsVariables(const GolfBallPhysicsVariables &) = delete;
    GolfBallPhysicsVariables &operator=(const GolfBallPhysicsVariables &) = delete;

    // Move constructor and assignment operator
    GolfBallPhysicsVariables(GolfBallPhysicsVariables &&) noexcept = default;
    GolfBallPhysicsVariables &operator=(GolfBallPhysicsVariables &&) noexcept = default;

    // Getters
    float getRhoImperial() const { return rhoImperial; }
    float getRhoMetric() const { return rhoMetric; }
    float getC0() const { return c0; }
    float getBeta() const { return beta; }
    float getV0() const { return v0_magnitude; }
    Vector3D getV0Vector() const { return v0.value_or(Vector3D{0, 0, 0}); }
    Vector3D getW() const { return w.value_or(Vector3D{0, 0, 0}); }
    float getOmega() const { return omega.value_or(0.0f); }
    float getROmega() const { return rOmega.value_or(0.0f); }
    float getTempC() const { return tempC; }
    float getElevationM() const { return elevationM; }
    Vector3D getVw() const { return vw.value_or(Vector3D{0, 0, 0}); }
    float getSVP() const { return SVP.value_or(0.0f); }
    float getBarometricPressure() const { return barometricPressure.value_or(0.0f); }
    float getRe100() const { return Re100.value_or(0.0f); }

    // Calculate all variables
    void calculateAllVariables();

private:
    // Private constructor, use Builder to create instances
    GolfBallPhysicsVariables(const golfBall &ball, const atmosphericData &atmos);

    const golfBall &ball;
    const atmosphericData &atmos;

    // Member variables
    float rhoImperial = 0.0f;
    float rhoMetric = 0.0f;
    float c0 = 0.0f;
    const float beta = 0.0001217f;
    float tempC = 0.0f;
    float elevationM = 0.0f;
    float v0_magnitude;

    std::optional<Vector3D> v0;
    std::optional<Vector3D> w;
    std::optional<Vector3D> vw;
    std::optional<float> omega;
    std::optional<float> rOmega;
    std::optional<float> SVP;
    std::optional<float> barometricPressure;
    std::optional<float> Re100;

    // Private calculation methods
    void calculateRhoMetric();
    void calculateRhoImperial();
    void calculateC0();
    void calculateV0();
    void calculateW();
    void calculateOmega();
    void calculateROmega();
    void calculateVw();
    void calculateSVP();
    void calculateBarometricPressure();
    void calculateRe100();
};

// Builder class definition
class GolfBallPhysicsVariables::Builder
{
public:
    Builder &setGolfBall(const golfBall &ball)
    {
        this->ball = &ball;
        return *this;
    }

    Builder &setAtmosphere(const atmosphericData &atmos)
    {
        this->atmos = &atmos;
        return *this;
    }

    GolfBallPhysicsVariables build() const
    {
        if (!ball || !atmos)
        {
            throw std::runtime_error("Golf ball and atmosphere data must be set");
        }
        return GolfBallPhysicsVariables(*ball, *atmos);
    }

private:
    const golfBall *ball = nullptr;
    const atmosphericData *atmos = nullptr;
};

#endif // GOLFBALLPHYSICSVARIABLES_HPP