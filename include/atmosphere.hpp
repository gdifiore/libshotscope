#ifndef ATMOSPHERE_HPP
#define ATMOSPHERE_HPP

#include <cstdint>

struct atmosphericData {
    uint8_t temp; // decimals better, but limited memory
    uint8_t elevation;
    uint8_t vWind;
    uint8_t phiWind; // angle of wind w.r.t Y-axis [-180, 180]; ex. 0deg -> CF, 45deg -> RF
    uint8_t hWind; // height where wind acts
    uint8_t relHumidity;
    float pressure; // Hg
};

#endif // ATMOSPHERE_HPP