#include "Control/PID.hpp"


PID::PID(float K_p, float K_i, float K_d, float k_ff, float time_const): 
    kp(K_p), ki(K_i), kd(K_d), kff(k_ff), tau(time_const), control_active(false), EPS_ON(0.14f), EPS_OFF(0.08f) {}

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
float PID::step(const float current){
    float error = setpoint - current;

    uint32_t now = millis();
    float dt = (now - last_time) / 1000.0;
    //compute P, I, and D values
    float integral_curr = 0.5 * dt * (last_error + error);
    float alpha = tau / (tau + dt);
    float filtered_derivative = alpha * last_derivative + (1 - alpha) * (error - last_error) / dt;
    float output = kp * error + ki * (integral_curr + integral) +  kd * filtered_derivative;
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

    if(hysteresisDeadband(error)) return 0;

    if(allow_integral && ki != 0)
        integral += integral_curr;

    return output;
}

/**
 * @brief Changes the signal saturation value.
 * 
 * @param[in] max Maximum signal value that PID is allowed to generate.
 * @param[in] min Minimum signal value that PID is allowed to generate.
 */
void PID::set_saturation(float max, float min){
    max_sat = max;
    min_sat = min;
}

/**
 * @brief Apply hysteresis deadband limit near the setpoint
 * 
 * @param[in] error reference to error that shows difference between setpoint and current value
 * @return A boolean, 0 outside deadband activate controller, 1 inside deadband deactivate controller.
 */
bool PID::hysteresisDeadband(float &error){
    if (control_active){
        // Turn OFF only when entering inner deadband
        if (abs(error) < EPS_OFF){
            control_active = false;
            return true;   // inside deadband → deactivate controller
        }
        return false;      // still active
    }else{
        // Stay OFF until error exceeds outer threshold
        if (abs(error) > EPS_ON){
            control_active = true;
            return false;  // activate controller
        }
        return true;       // remain inside deadband
    }
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

void PID::setSetpoint(float sp){
    control_active = true;
    setpoint = sp;
}