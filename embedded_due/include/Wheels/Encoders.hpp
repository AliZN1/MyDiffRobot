#ifndef ENCODERS_H
#define ENCODERS_H

#include <cstdint>
#include <Arduino.h>
#include <Config.hpp>
#include "TaskManager/Task.hpp"
#include <SerialComManager/SerialComManager.hpp>


using namespace std;
class SerialManager;

#define omega_min_dt 150 // ms

// ------------------------------------ Encoder
class Encoder{
private:
    uint8_t pin;
    uint16_t uperLimit;
    uint16_t lowerLimit;
    float omega; //angular velocity
    double last_angPos;
    double last_angle;
    int32_t numTurns;
    double initAngle;
    int8_t dir;
public:
    Encoder(uint8_t pin_num, bool reverseDir = false);
    ~Encoder();
    void initLastAngle();
    double readAngle();
    double updateAngDisp();
    void computeOmega(uint32_t &dt, double &cur_angPos);
    void setLimits(uint16_t min, uint16_t max);
    float getOmega();
};

// ------------------------------------ EncoderManager
class EncodersManager: public Task{
private:
    Encoder enc_R;
    Encoder enc_L;
    SerialManager &serialManager;
    uint32_t last_omegaStamp;
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
    void angularPos(double (&angPosArr)[2]);
    void initLastAngles();
    void getAngVel(float (&angVel)[2]); // return right and left omega respectively
};

#endif