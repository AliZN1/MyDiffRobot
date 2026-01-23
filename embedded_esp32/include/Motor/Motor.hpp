#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>
#include "Config.hpp"
#include "Control/PID.hpp"


#define motor_PID_Kp 10
#define motor_PID_Ki 0
#define motor_PID_Kd 0.4
#define motor_PID_Kff 0 // feed forward gain
#define motor_saturation_max 254
#define motor_saturation_min -254
#define wheelRad_axleLen 0.53719 // wheel radius over axle length


class Motor {
private:
    uint8_t pin_A;
    uint8_t pin_B;
    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    
public:
    PID pid;
    Motor(const uint8_t pinA, const uint8_t pinB);
    ~Motor();
    void move(int16_t speed);
    void stop();
};


#endif