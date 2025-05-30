#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <cstdint>
#include <Arduino.h>


class PID{
private:
    float kp, ki, kd, tau;
    double max_sat;
    double min_sat;
    double integral;
    uint32_t last_time;
    double last_error;
    double last_derivative;
    double setpoint;
public:
    PID(float kp, float ki, float kd, float time_const = 0.02);
    ~PID();
    double run(const double current);
    void set_saturation(double max, double min);
    void update_setpoint(double newSetpoint);
    void reset();
};

#endif