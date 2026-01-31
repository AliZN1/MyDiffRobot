#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <Arduino.h>
#include "Config.hpp"



class Motor {
private:
    uint8_t pin_A;
    uint8_t pin_B;
    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    
public:
    Motor(const uint8_t pinA, const uint8_t pinB);
    ~Motor();
    void move(int16_t speed);
    void stop();
};


#endif