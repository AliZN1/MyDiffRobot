#include "Motor/MotorsController.hpp"


MotorsController::MotorsController(uint8_t (&mdp)[4], QueueHandle_t &encoders_data_q, QueueHandle_t &motion_q)
    : motor_R(mdp[0], mdp[1]), motor_L(mdp[2], mdp[3]), encoders_q(encoders_data_q), motion_cmd_q(motion_q) {}

MotorsController::~MotorsController() {}

void MotorsController::runTask(){
    EncoderData_t enc_data;
    MotionType_t motion;

    while(1){
        vTaskDelay(pdMS_TO_TICKS(motorsController_d));

        if(xQueueReceive(motion_cmd_q, &motion, pdMS_TO_TICKS(0)) == pdPASS){
            switch (motion.type)
            {
            case angular_deg:
                motor_R.pid.update_setpoint(deg2rad(motion.value*2));
                motor_L.pid.update_setpoint(deg2rad(motion.value*2));
                break;
            case linear_dist:
                motor_R.pid.update_setpoint(deg2rad(motion.value));
                motor_L.pid.update_setpoint(deg2rad(-1*motion.value));
                break;
            default:
                break;
            }
            controller_running = true;
        }

        switch (motion.type)
        {
        case angular_deg:
            rotate();
            break;
        case linear_dist:
            rotate();
            break;
        default:
            break;
        }
    }
}

void MotorsController::rotate(){
    EncoderData_t enc_data;

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
}


float MotorsController::deg2rad(float ang){
    return ang * PI / 180;
}

float dist2rad(float dist){
    
}