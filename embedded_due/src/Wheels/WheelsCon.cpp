#include "Wheels/WheelsCon.hpp"

// Wheel interface class
Wheel::Wheel(const uint8_t pinA, const uint8_t pinB):
    pin_A(pinA), pin_B(pinB), pid(wheel_PID_Kp, wheel_PID_Ki, wheel_PID_Kd, 0.01) {
        pid.set_saturation(254, 50);
    }

Wheel::~Wheel(){}

void Wheel::moveForward(const uint8_t speed){
    analogWrite(pin_A, 0);
    analogWrite(pin_B, speed);
}

void Wheel::moveBackward(const uint8_t speed){
    analogWrite(pin_A, speed);
    analogWrite(pin_B, 0);
}

// void Wheel::move(int16_t speed){
//     if(speed > 0)
//         moveForward(speed);
//     else if(speed < 0)
//         moveBackward(speed);
//     else
//         stop();
// }

void Wheel::stop(){
    analogWrite(pin_A, 0);
    analogWrite(pin_B, 0);
}

// wheels controller class
WheelsCon::WheelsCon(uint8_t (&motorDriverPins)[4], EncodersManager &encodersManager, uint16_t num_tick):
    Task(num_tick), wheel_R(motorDriverPins[0], motorDriverPins[1]), wheel_L(motorDriverPins[2], motorDriverPins[3]), encodersM(encodersManager) {}

WheelsCon::~WheelsCon(){}

void WheelsCon::execute(){
    float angVel[2];
    encodersM.getAngVel(angVel);

    double velCommand_R = wheel_R.pid.run(angVel[0]);
    double velCommand_L = wheel_L.pid.run(angVel[1]);
    wheel_R.moveForward((uint8_t)velCommand_R);
    wheel_L.moveForward((uint8_t)velCommand_L);

    // Serial.print("speed: ");
    // Serial.print(angVel[0]);
    // Serial.print(", cmd: ");
    // Serial.println(velCommand_R);
}

void WheelsCon::moveFW(const uint8_t speed){
    wheel_R.pid.update_setpoint(speed);
    wheel_L.pid.update_setpoint(speed);

    // wheel_R.moveForward(speed);
    // wheel_L.moveForward(speed);
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