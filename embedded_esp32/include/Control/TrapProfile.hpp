/*Trapezoidal profile is a motion profiling that dictates a smooth trajectory for a system by constraining acceleration, velocity, and deceleration into a trapezoidal velocity-time graph. */

#pragma once

#include <Arduino.h>

#define POS_EPS_RAD    0.01f // rad
#define VEL_EPS_RPS    0.1f // rad/s 

class TrapProfile{
private:
    float w_max, a_max;
    float dt;
    float pos_eps, vel_eps;
    float theta_target, theta_ref, omega_ref;

public:
    TrapProfile(float time_step, float pos_epsilon = POS_EPS_RAD, float vel_epsilon = VEL_EPS_RPS);
    ~TrapProfile();
    void reset(float current, float target);
    float step();
    bool finished();
    void set_max_vel_accel(float velMax, float accelMax);
    void set_epsilons(float pos_epsilon, float vel_epsilon);
};