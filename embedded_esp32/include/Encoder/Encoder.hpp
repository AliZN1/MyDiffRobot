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
    float omega; //angular velocity
    float last_angPos;
    float last_angle;
    int32_t numTurns;
    float initAngle;
    int8_t dir;
public:
    Encoder(uint8_t pin_num, bool reverseDir = false);
    ~Encoder();
    void initLastAngle();
    float readAngle();
    float updateAngDisp();
    void computeOmega(uint32_t &dt, float &cur_angPos);
    void setLimits(uint16_t min, uint16_t max);
    float getOmega();
};

#endif