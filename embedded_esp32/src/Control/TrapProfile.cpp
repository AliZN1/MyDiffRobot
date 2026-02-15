#include "Control/TrapProfile.hpp"


TrapProfile::TrapProfile(float time_step, float pos_epsilon, float vel_epsilon)
    : dt(time_step), pos_eps(pos_epsilon), vel_eps(vel_epsilon) {}

TrapProfile::~TrapProfile(){}

float TrapProfile::step(){
    const float dist = theta_target - theta_ref;  // remaining distance

    // Serial.print(theta_target);
    // Serial.print(", ");
    // Serial.print(theta_ref);
    // Serial.print(", ");

    float d_brake = omega_ref * omega_ref / (2 * a_max);
    float dir = (dist > 0) ? -1.0f : 1.0f;
    
    if(fabs(dist) < d_brake)
        omega_ref += dir * a_max * dt;
    else
        omega_ref -= dir * a_max * dt;

    omega_ref = constrain(omega_ref, -w_max, w_max);

    // Serial.print(",");
    // Serial.println(omega_ref);
    theta_ref += omega_ref * dt;

    // Serial.println(omega_ref);

    if (fabs(dist) < 0.05) {
        theta_ref = theta_target;
        omega_ref = 0.0f;
    }

    return theta_ref;
}

void TrapProfile::reset(float current, float target){
    theta_ref = current;
    omega_ref = 0;
    theta_target = target;
}

bool TrapProfile::finished(){
    return (fabs(theta_target - theta_ref) <= pos_eps);
}

void TrapProfile::set_max_vel_accel(float velMax, float accelMax){
    w_max = velMax;
    a_max = accelMax;
}

void TrapProfile::set_epsilons(float pos_epsilon, float vel_epsilon){
    pos_eps = pos_epsilon;
    vel_eps = vel_epsilon;
}