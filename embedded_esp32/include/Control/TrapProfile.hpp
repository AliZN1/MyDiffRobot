/*Trapezoidal profile is a motion profiling that dictates a smooth trajectory for a system by constraining acceleration, velocity, and deceleration into a trapezoidal velocity-time graph. */

#pragma once

#include <Arduino.h>

#define POS_EPS_RAD    0.02f // rad
#define VEL_EPS_RPS    0.02f // rad/s 

class TrapProfile{
private:
    float q; // current profile position (rad)
    float v; // current profile velocity (rad/s)
    float vmax;
    float amax;
    float pos_eps;
    float vel_eps;
    float q_target;
    static float clampf(float x, float min, float max);
public:
    TrapProfile(float pos_epsilon = POS_EPS_RAD, float vel_epsilon = VEL_EPS_RPS);
    ~TrapProfile();
    void reset(float q0, float q_targ);
    float step(float dt);
    bool finished();
    void set_max_vel_accel(float velMax, float accelMax);
    void set_epsilons(float pos_epsilon, float vel_epsilon);
};