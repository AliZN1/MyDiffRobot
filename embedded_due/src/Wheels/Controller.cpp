#include "Wheels/Controller.hpp"


PID::PID(float K_p, float K_i, float K_d, float k_ff, float time_const): 
    kp(K_p), ki(K_i), kd(K_d), kff(k_ff), tau(time_const), setpoint(0) {}

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
double PID::run(const double current){
    double error = setpoint - current;
    uint32_t now = millis();
    float dt = (now - last_time) / 1000.0;
    //compute P, I, and D values
    double integral_curr = 0.5 * dt * (last_error + error);
    double alpha = tau / (tau + dt);
    double filtered_derivative = alpha * last_derivative + (1 - alpha) * (error - last_error) / dt;
    double output = kp * error + ki * (integral_curr + integral) +  kd * filtered_derivative;
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
    if(allow_integral && ki != 0)
        integral += integral_curr;
    //store data that is used in the next step
    last_error = error;
    last_derivative = filtered_derivative;
    last_time = now;

    return output;
}

/**
 * @brief Changes the signal saturation value.
 * 
 * @param[in] max Maximum signal value that PID is allowed to generate.
 * @param[in] min Minimum signal value that PID is allowed to generate.
 */
void PID::set_saturation(double max, double min){
    max_sat = max;
    min_sat = min;
}

/**
 * @brief Sets a new reference value for the PID to reach.
 * 
 * @param[in] newSetPoint New set point for PID to reach.
 */
void PID::update_setpoint(double newSetpoint){
    setpoint = newSetpoint;
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
    setpoint = 0;
}
