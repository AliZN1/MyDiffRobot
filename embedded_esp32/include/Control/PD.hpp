#pragma once

#include <Arduino.h>
#include "Control/P.hpp"

class PD: public P{
private:
    float kd, tau;
    float last_error;
    float last_derivative;

public:
    PD(float kp, float kd, float k_ff=0.0, float time_const=0.02);
    ~PD();
    float step(const float &setpoint, const float &current) override;
    void reset() override;
};