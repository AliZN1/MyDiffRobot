#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <Arduino.h>


class PID{
private:
    float kp, ki, kd, kff, tau;
    float max_sat;
    float min_sat;
    float integral;
    uint32_t last_time;
    float last_error;
    float last_derivative;
    float setpoint;
    const float EPS_ON, EPS_OFF;
public:
    bool control_active;
    PID(float kp, float ki, float kd, float k_ff=0.0, float time_const=0.02);
    ~PID();
    float step(const float current);
    void set_saturation(float max, float min);
    bool hysteresisDeadband(float &error);
    void reset();
    void setSetpoint(float sp);
};

#endif