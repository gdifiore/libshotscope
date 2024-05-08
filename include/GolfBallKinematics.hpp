#ifndef GOLFBALLKINEMATICS_HPP
#define GOLFBALLKINEMATICS_HPP

class GolfBallKinematics
{
public:
    GolfBallKinematics();

    void calculateAllVariables();

    static constexpr float dt = 0.01;

    float getTime() const { return t; }
    float getX() const { return x; }
    float getY() const { return y; }
    float getZ() const { return z; }

private:

    float t;
    float x;
    float y;
    float z;

    void calcTime();
    void calcX();
    void calcY();
    void calcZ();
};

#endif // GOLFBALLKINEMATICS_HPP