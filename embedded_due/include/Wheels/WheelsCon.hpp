#ifndef WHEELSCON_HPP
#define WHEELSCON_HPP

#include <Arduino.h>
#include <cstdint>
#include "Wheels/WheelsCon.hpp"
#include "Wheels/Controller.hpp"
#include "Wheels/Encoders.hpp"
#include "TaskManager/Task.hpp"

#define wheel_PID_Kp 10
#define wheel_PID_Ki 1
#define wheel_PID_Kd 0
#define wheel_PID_Kff 15 // feed forward gain
#define motor_saturation_max 254
#define motor_saturation_min -254

class EncodersManager;

class Wheel{
private:
    uint8_t pin_A;
    uint8_t pin_B;
public:
    PID pid;
    Wheel(const uint8_t pinA, const uint8_t pinB);
    ~Wheel();
    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    void move(int16_t speed);
    void stop();
};

class WheelsCon: public Task{
private:
    Wheel wheel_R;
    Wheel wheel_L;
    EncodersManager &encodersM;
    bool stopped;
protected:
    void execute() override;
public:
    WheelsCon(uint8_t (&motorDriverPins)[4], EncodersManager &encodersManager, uint16_t num_tick);
    ~WheelsCon();
    void move_cmd(int8_t speed);
    void rotateCW(uint8_t speed);
    void rotateCCW( uint8_t speed);
    void stop();
};


#endif // MACRO
