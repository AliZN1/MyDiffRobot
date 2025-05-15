#ifndef ENCODERS_H
#define ENCODERS_H

#include <cstdint>
#include <Arduino.h>
#include <Config.hpp>
#include <SerialComManager/SerialComManager.hpp>
#include "TaskManager/Task.hpp"

using namespace std;

// ------------------------------------ Encoder
class Encoder{
private:
    uint8_t pin;
    uint16_t uperLimit;
    uint16_t lowerLimit;
    double last_angle;
    int32_t numTurns;
    double initAngle;
    int8_t dir;
public:
    Encoder(uint8_t pin_num, bool reverseDir = false);
    ~Encoder();
    double readAngle();
    double updateAngDisp();
    void setLimits(uint16_t min, uint16_t max);
    void initLastAngle();
};

// ------------------------------------ EncoderManager
class EncodersManager: public Task{
private:
    Encoder enc_R;
    Encoder enc_L;
    SerialManager &serialManager;
protected:
    void execute() override;
public:
    EncodersManager(
        uint8_t pin_R,
        uint8_t pin_L,
        SerialManager &sm,
        uint16_t num_ticks
    );
    ~EncodersManager();
    void angularPos();
    void initLastAngles();
};

#endif