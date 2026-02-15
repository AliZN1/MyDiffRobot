#include "Control/PID.hpp"


PID::PID(float K_p, float K_i, float K_d, float K_ff, float time_const): 
    P(K_p, K_ff), ki(K_i), kd(K_d), tau(time_const) { }

PID::~PID(){}

/**
 * @brief Executes PID logic based on current value and returns controller signal.
 *
 * The controller has all three elements of PID control. 
 * Derivative part has low-pass filter. Additionally, the controller is equipped 
 * with anti-windup, feed-forward, and saturation control. PID and feed-forward 
 * gains as well as low-pass filter time constant must be defined in class 
 * constructor. The saturation limit must be applied using `set_saturation` 
 * function.
 * 
 * @param[in] current current value read from sensor.
 * @return control signal for the actuator(s).
 */
float PID::step(const float &setpoint, const float &current){
    float error = setpoint - current;

    uint32_t now = millis();
    float dt = (now - last_time) / 1000.0;
    //compute P, I, and D values
    float integral_curr = 0.5 * dt * (last_error + error);
    float alpha = tau / (tau + dt);
    float filtered_derivative = alpha * last_derivative + (1 - alpha) * (error - last_error) / dt;
    float output = kp * error + ki * (integral_curr + integral) +  kd * filtered_derivative;
    hysteresisDeadband(error, output);
    output += kff * setpoint;
    //apply saturation and anti-windup
    bool allow_integral = true;
    if(output > max_sat){
        output = max_sat;
        if(error > 0) allow_integral = false;
    }
    else if(output < min_sat){
        output = min_sat;
        if(error < 0) allow_integral = false;
    }
    //store data that is used in the next step
    last_error = error;
    last_derivative = filtered_derivative;
    last_time = now;

    if(allow_integral && ki != 0)
        integral += integral_curr;

    return output;
}

/**
 * @brief Resets the PID and changes the set point to `0`.
 * 
 * All internal values used to compute integral and difference are set to `0`.
 */
void PID::reset(){
    last_time = 0;
    last_error = 0;
    integral = 0;
    last_derivative = 0;
    control_active = false;
}
