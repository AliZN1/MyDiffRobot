#ifndef ENCODERS_H
#define ENCODERS_H

#include <cstdint>
#include <Arduino.h>
#include <Config.hpp>
#include <stdlib.h>

using namespace std;

// ------------------------------------ Encoder
class Encoder{
private:
    uint8_t pin;
    uint16_t uperLimit;
    uint16_t lowerLimit;
    double last_angle;
    uint32_t numTurns;
public:
    Encoder(uint8_t pin_num);
    ~Encoder();
    double readAngle();
    void setLimits(uint16_t min, uint16_t max);
    double updateAngDisp();
};

// ------------------------------------ EncoderManager
class EncodersManager{
private:
    Encoder enc_R;
    Encoder enc_L;
public:
    EncodersManager(
        uint8_t pin_R,
        uint8_t pin_L
    );
    ~EncodersManager();
    void angularPos();
};

#endif