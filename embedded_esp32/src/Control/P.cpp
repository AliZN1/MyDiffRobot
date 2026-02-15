#include "Control/P.hpp"


P::P(float K_p, float k_ff): 
    kp(K_p), kff(k_ff), control_active(false), deadband_on(true), EPS_ON(0.1f), EPS_OFF(0.2f) {}

P::~P(){}

float P::step(const float &setpoint, const float &current){
    float error = setpoint - current;

    uint32_t now = millis();
    float dt = (now - last_time) / 1000.0;
    //compute P values
    float output = kp * error;
    hysteresisDeadband(error, output);
    output += kff * setpoint;
    //apply saturation
    if(output > max_sat)
        output = max_sat;
    else if(output < min_sat)
        output = min_sat;
    //store data that is used in the next step
    last_time = now;

    return output;
}

/**
 * @brief Changes the signal saturation value.
 * 
 * @param[in] max Maximum signal value that PID is allowed to generate.
 * @param[in] min Minimum signal value that PID is allowed to generate.
 */
void P::setSaturation(float max, float min){
    max_sat = max;
    min_sat = min;
}

/**
 * @brief Configures the deadband thresholds for controller activation.
 *
 * The controller is deactivated when the absolute error is below esp_off
 * and activated when the absolute error exceeds esp_on. If eps_on = eps_off = 0, 
 * then deadband is deactivated.
 *
 * @param[in] eps_off Absolute error threshold below which the controller is deactivated.
 * @param[in] eps_on  Absolute error threshold above which the controller is activated.
 */

void P::setDeadband(float eps_on, float eps_off){
    EPS_ON = eps_on;
    EPS_OFF = eps_off;

    if(eps_on == 0.0 && eps_off == 0.0)
        deadband_on = false;
    else
        deadband_on = true;

}

/**
 * @brief Apply hysteresis deadband limit near the setpoint
 * 
 * @param[in] error reference to error that shows difference between setpoint and current value
 * @return A boolean, 0 outside deadband activate controller, 1 inside deadband deactivate controller.
 */
void P::hysteresisDeadband(const float &error, float &output){
    if(!deadband_on)
        return;

    if (control_active){
        // Turn OFF only when entering inner deadband
        if (abs(error) < EPS_OFF){
            control_active = false;
            output = 0.0f; // inside deadband → deactivate controller
        }
        // else still active
    }else{
        // Stay OFF until error exceeds outer threshold
        if (abs(error) > EPS_ON){
            control_active = true; // activate controller
        }
        else 
            output = 0.0f; // remain inside deadband
    }
}

/**
 * @brief Resets the PID and changes the set point to `0`.
 * 
 * All internal values used to compute integral and difference are set to `0`.
 */
void P::reset(){
    last_time = 0;
    control_active = false;
}