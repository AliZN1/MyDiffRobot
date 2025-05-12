#ifndef WHEELSCON_HPP
#define WHEELSCON_HPP

#include <Arduino.h>
#include <cstdint>
#include "Wheels/WheelsCon.hpp"

class Wheel{
private:
    uint8_t pin_A;
    uint8_t pin_B;
public:
    Wheel(const uint8_t pinA, const uint8_t pinB);
    ~Wheel();
    void moveForward(const uint8_t speed);
    void moveBackward(const uint8_t speed);
    void stop();
};

class WheelsCon{
private:
    Wheel wheel_R;
    Wheel wheel_L;
public:
    WheelsCon(const uint8_t m1_A, const uint8_t m1_B, const uint8_t m2_A, const uint8_t m2_B);
    ~WheelsCon();
    void moveFW(const uint8_t speed);
    void moveBW(const uint8_t speed);
    void stop();
};


#endif // MACRO
