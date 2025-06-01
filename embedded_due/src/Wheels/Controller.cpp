#include "Wheels/Controller.hpp"


PID::PID(float K_p, float K_i, float K_d, float k_ff, float time_const): 
    kp(K_p), ki(K_i), kd(K_d), kff(k_ff), tau(time_const), setpoint(0) {}

PID::~PID(){}

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

void PID::set_saturation(double max, double min){
    max_sat = max;
    min_sat = min;
}

void PID::update_setpoint(double newSetpoint){
    setpoint = newSetpoint;
}

void PID::reset(){
    last_time = 0;
    last_error = 0;
    integral = 0;
    last_derivative = 0;
    setpoint = 0;
}
