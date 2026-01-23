#include "Motor/Motor.hpp"


Motor::Motor(const uint8_t pinA, const uint8_t pinB):pin_A(pinA), pin_B(pinB), pid(motor_PID_Kp, motor_PID_Ki, motor_PID_Kd, motor_PID_Kff, 0.01) {
        pid.set_saturation(motor_saturation_max, motor_saturation_min);
}

Motor::~Motor(){}

/**
 * @brief This function runs the process the motor is required to do.
 */
// void Motor::runTask(){
//     int16_t sp_deg;
//     float angPos;

//     while(1){
//         if(xQueueReceive(setpoint_q, &sp_deg, 0) == pdPASS)
//             pid.update_setpoint(deg2rad(sp_deg));
        
//         if(xQueueReceive(encoder_q, &angPos, 10) == pdPASS){
//             float command = pid.run(angPos);
//             if(command == 0) stop();
//             else if(command > 0) move(command + 190);
//             else move(command - 190);
//         }
//         else
//             move(0);

//         vTaskDelay(pdMS_TO_TICKS(CONTROL_TICK));
//     }
// }

/**
 * @brief Rotates the motor clockwise at given speed.
 * 
 * @param[in] speed A PWM signal (between 0 to 255) sent to the motor driver.
 */
void Motor::moveForward(uint8_t speed){
    analogWrite(pin_A, 0);
    analogWrite(pin_B, speed);
}

/**
 * @brief Rotates the motor counterclockwise at given speed.
 * 
 * @param[in] speed A PWM signal (between 0 to 255) sent to the motor driver.
 */
void Motor::moveBackward(uint8_t speed){
    analogWrite(pin_A, speed);
    analogWrite(pin_B, 0);
}

/**
 * @brief Rotates the motor clockwise/counterclockwise at given speed.
 * 
 * @param[in] speed A PWM signal (between 0 to 255) sent to the motor driver.
 *      The sign of the speed determines the direction: positive for clockwise, 
 *      negative for counterclockwise.
 */
void Motor::move(int16_t speed){
    if (speed == 0) stop();
    else if(speed > 254) speed = 254;
    else if(speed < -254) speed = -254;

    if(speed > 0) moveForward(speed);
    else if(speed < 0) moveBackward(abs(speed));
}

/**
 * @brief Stops the motor from moving.
 * 
 * It resets the motor associated PID speed controller.
 */
void Motor::stop(){
    // pid.update_setpoint(0);
    // pid.reset();
    analogWrite(pin_A, 0);
    analogWrite(pin_B, 0);
}