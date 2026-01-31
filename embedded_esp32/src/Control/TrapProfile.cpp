#include "Control/TrapProfile.hpp"


TrapProfile::TrapProfile(float pos_epsilon, float vel_epsilon)
    : pos_eps(pos_epsilon), vel_eps(vel_epsilon) {}

TrapProfile::~TrapProfile(){}

float TrapProfile::step(float dt){
    const float e = q_target - q;               // remaining distance
    const float dir = (e >= 0) ? 1 : -1;

    // braking distance for current speed: d = v^2 / (2 a)
    const float v_abs = abs(v);
    const float d_brake = (amax > 0) ? (v_abs * v_abs) / (2.0f * amax) : 0;

    // Choose acceleration: speed up if far enough, else brake
    float a = 0;
    if (abs(e) > d_brake) {
        a = dir * amax; // accelerate toward target
    } else {
        a = -dir * amax; // brake to stop at target
    }

    // integrate velocity with saturation
    v += a * dt;
    v = clampf(v, -vmax, vmax);

    // integrate position
    q += v * dt;

    // If we overshoot or get extremely close, snap to target and stop
    // (prevents endless dithering due to discrete dt)
    const float e_new = q_target - q;
    if ((dir > 0 && e_new <= 0) || (dir < 0 && e_new >= 0)) {
        q = q_target;
        v = 0;
    }

    return q;
}

void TrapProfile::reset(float q0, float q_targ){
    q = q0;
    v = 0;
    q_target = q_targ;
}

bool TrapProfile::finished(){
    return (abs(q_target - q) <= pos_eps) && (abs(v) <= vel_eps);
}

float TrapProfile::clampf(float x, float min, float max){
    return(x < min) ? min : (x > max) ? max : x;
}

void TrapProfile::set_max_vel_accel(float velMax, float accelMax){
    vmax = velMax;
    amax = accelMax;
}

void TrapProfile::set_epsilons(float pos_epsilon, float vel_epsilon){
    pos_eps = pos_epsilon;
    vel_eps = vel_epsilon;
}