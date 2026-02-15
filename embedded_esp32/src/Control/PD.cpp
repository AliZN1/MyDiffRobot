#include "Control/PD.hpp"


PD::PD(float K_p, float K_d, float K_ff, float time_const): 
    P(K_p, K_ff), kd(K_d), tau(time_const) { }

PD::~PD(){}

float PD::step(const float &setpoint, const float &current){
    float error = setpoint - current;

    uint32_t now = millis();
    float dt = (now - last_time) / 1000.0;
    //compute P and D values
    float alpha = tau / (tau + dt);
    float filtered_derivative = alpha * last_derivative + (1 - alpha) * (error - last_error) / dt;
    float output = kp * error +  kd * filtered_derivative;
    hysteresisDeadband(error, output);
    output += kff * setpoint;
    //apply saturation
    if(output > max_sat)
        output = max_sat;
    else if(output < min_sat)
        output = min_sat;
    //store data that is used in the next step
    last_error = error;
    last_derivative = filtered_derivative;
    last_time = now;

    return output;
}

/**
 * @brief Resets the PD and changes the set point to `0`.
 * 
 * All internal values used to compute integral and difference are set to `0`.
 */
void PD::reset(){
    last_time = 0;
    last_error = 0;
    last_derivative = 0;
    control_active = false;
}