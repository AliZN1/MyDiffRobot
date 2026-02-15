#ifndef ENCODER
#define ENCODER

// #include <cstring>
#include <Arduino.h>
#include "Config.hpp"


class Encoder{
private:
    uint8_t pin;
    uint16_t uperLimit;
    uint16_t lowerLimit;
    float last_angle;
    float continuousAngle;
    int8_t dir;
public:
    Encoder(uint8_t pin_num, bool reverseDir = false);
    ~Encoder();
    float readAngle();
    float updateAngDisp();
    void setLimits(uint16_t min, uint16_t max);
};

#endif