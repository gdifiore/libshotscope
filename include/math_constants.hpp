#ifndef MATH_CONSTANTS_HPP
#define MATH_CONSTANTS_HPP

#include <cstdint>

namespace MATH_CONSTANTS {
    /*
     * The below constants were taken from a study at Washington State University by Bin Lyu, et al.
     *
     * 3 different Coefficient of drag (Cd) values, determined based on the calculated Reynolds number
     *
     * Cd = CdL for Re<=0.5e5
     * Cd = CdL+(CdL-CdH)*(Re*1e-5-1)/0.5 for Re>0.5 and Re<1.0
     * Cd = CdH for Re>=1.0e5
    */
    constexpr float CdS = 0.180;
    constexpr float CdL = 0.500;
    constexpr float CdH = 0.200;
    /*
     * 2 coefficients [coeff(1/2)] are used to calculate the lift coefficient (Cl)
     *
     * Cl = coeff1*S+coeff2*S^2 for S<=0.3 and =0.305 for S>0.3
     * where S is the non-dimensional spin factor
     */
    constexpr float Cl_default = 0.305;
    constexpr float coeff1 = 1.990;
    constexpr float coeff2 = -3.250;

    // some const for testing
    constexpr float someconst = 5.682E-03;
}

#endif // MATH_CONSTANTS_HPP