#ifndef ATMOSPHERE_HPP
#define ATMOSPHERE_HPP

struct atmosphericData {
    float temp; // decimals better, but limited memory
    float elevation;
    float vWind;
    float phiWind; // angle of wind w.r.t Y-axis [-180, 180]; ex. 0deg -> CF, 45deg -> RF
    float hWind; // height where wind acts
    float relHumidity;
    float pressure; // Hg
};

#endif // ATMOSPHERE_HPP