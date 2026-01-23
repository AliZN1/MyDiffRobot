#include "Motor/MotorsManager.hpp"


MotorsManager::MotorsManager(uint8_t (&mdp)[4], QueueHandle_t &encoders_data_q, QueueHandle_t &motor_setpoints_q)
    : motor_R(mdp[0], mdp[1]), motor_L(mdp[2], mdp[3]), encoders_q(encoders_data_q), setpoints_q(motor_setpoints_q) {}

MotorsManager::~MotorsManager() {}

void MotorsManager::runTask(){
    MotorSetpoint_t setpoints;
    EncoderData_t enc_data;

    while(1){
        if(xQueueReceive(setpoints_q, &setpoints, 0) == pdPASS){
            motor_R.pid.update_setpoint(deg2rad(setpoints.right));
            motor_L.pid.update_setpoint(deg2rad(setpoints.left));
        }
        
        if(xQueueReceive(encoders_q, &enc_data, 10) == pdPASS){
            float command_r = motor_R.pid.run(enc_data.right);
            float command_l = motor_L.pid.run(enc_data.left);

            if(command_r == 0) motor_R.stop();
            else if(command_r > 0) motor_R.move(command_r + 190);
            else motor_R.move(command_r - 190);

            if(command_l == 0) motor_L.stop();
            else if(command_l > 0) motor_L.move(command_l + 190);
            else motor_L.move(command_l - 190);
        }
        else{
            motor_R.move(0);
            motor_L.move(0);
        }

        vTaskDelay(pdMS_TO_TICKS(CONTROL_TICK));
    }
}

float MotorsManager::deg2rad(float ang){
    return ang * PI / 180;
}