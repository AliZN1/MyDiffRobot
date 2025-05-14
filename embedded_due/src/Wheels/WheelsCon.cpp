#include "Wheels/WheelsCon.hpp"

// Wheel interface class
Wheel::Wheel(const uint8_t pinA, const uint8_t pinB):
    pin_A(pinA), pin_B(pinB) {}

Wheel::~Wheel(){}

void Wheel::moveForward(const uint8_t speed){
    analogWrite(pin_A, 0);
    analogWrite(pin_B, speed);
}

void Wheel::moveBackward(const uint8_t speed){
    analogWrite(pin_A, speed);
    analogWrite(pin_B, 0);
}

void Wheel::stop(){
    analogWrite(pin_A, 0);
    analogWrite(pin_B, 0);
}

// wheels controller class
WheelsCon::WheelsCon(const uint8_t m1_A, const uint8_t m1_B, const uint8_t m2_A, const uint8_t m2_B):
    wheel_R(m1_A, m1_B), wheel_L(m2_A, m2_B) {}

WheelsCon::~WheelsCon(){}

void WheelsCon::moveFW(const uint8_t speed){
    wheel_R.moveForward(speed);
    wheel_L.moveForward(speed);
}

void WheelsCon::moveBW(const uint8_t speed){
    wheel_R.moveBackward(speed);
    wheel_L.moveBackward(speed);
}

void WheelsCon::rotateCW(const uint8_t speed){
    wheel_R.moveBackward(speed);
    wheel_L.moveForward(speed);
}

void WheelsCon::rotateCCW(const uint8_t speed){
    wheel_R.moveForward(speed);
    wheel_L.moveBackward(speed);
}

void WheelsCon::stop(){
    wheel_R.stop();
    wheel_L.stop();
}