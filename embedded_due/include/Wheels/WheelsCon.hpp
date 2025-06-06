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
#define PID_R_kp 0.1
#define PID_R_ki 0.1
#define PID_R_kd 0
#define motor_saturation_max 254
#define motor_saturation_min -254
#define wheelRad_axleLen 0.53719 // wheel radius over axle length


class EncodersManager;
class IMU;

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
    IMU &imu;
    PID pid_rotation;
    double motor_yawVel;
    int16_t headAng_setpoint;
    uint8_t i;
    bool stopped;
    bool R_stopped;
protected:
    void execute() override;
public:
    WheelsCon(uint8_t (&motorDriverPins)[4], EncodersManager &encodersManager, IMU &imu, uint16_t num_tick);
    ~WheelsCon();
    void set_linearVel(int8_t speed);
    void driveLinear(float (&angVel)[2]);
    void set_angularPos(uint16_t speed);
    void driveAngular(float cur_angle);
    void stop();
};


#endif // MACRO
