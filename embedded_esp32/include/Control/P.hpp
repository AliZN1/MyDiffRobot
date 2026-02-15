#pragma once

#include <Arduino.h>

class P{
protected:
    float kp, kff;
    float max_sat;
    float min_sat;
    uint32_t last_time;
    bool deadband_on;
    float EPS_ON, EPS_OFF;

public:
    bool control_active;
    P(float kp, float k_ff=0.0);
    ~P();
    virtual float step(const float &setpoint, const float &current);
    virtual void reset();
    void setSaturation(float max, float min);
    void setDeadband(float eps_on, float eps_off);
    void hysteresisDeadband(const float &error, float &output);
};