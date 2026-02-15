#include "Motor/MotorsController.hpp"


MotorsController::MotorsController(uint8_t (&mdp)[4], QueueHandle_t &encoders_data_q, QueueHandle_t &motion_q)
    : motor_R(mdp[0], mdp[1]), 
    motor_L(mdp[3], mdp[2]), 
    pd_pos_R(pos_PID_Kp, pos_PID_Kd, pos_PID_Kff, time_constant),
    pd_pos_L(pos_PID_Kp, pos_PID_Kd, pos_PID_Kff, time_constant),
    trap_L(motorsController_d*0.001),
    trap_R(motorsController_d*0.001),
    encoders_q(encoders_data_q), 
    motion_cmd_q(motion_q)
{
    pd_pos_R.setSaturation(motor_saturation_max, motor_saturation_min);
    pd_pos_L.setSaturation(motor_saturation_max, motor_saturation_min);
    pd_pos_R.setDeadband(0.03, 0.05);
    pd_pos_L.setDeadband(0.03, 0.05);

    trap_L.set_epsilons(trap_pos_eps, trap_vel_eps);
    trap_R.set_epsilons(trap_pos_eps, trap_vel_eps);
    
    enc_data.right = 0.0;
    enc_data.left = 0.0;
}

MotorsController::~MotorsController() {}

void MotorsController::runTask(){
    state = State::IDLE;

    // Initialize to current position
    if(xQueueReceive(encoders_q, &enc_data, 10) == pdPASS)
        Serial.println("no data available from enc!");

    pd_pos_R.reset();
    pd_pos_L.reset();

    MotionType_t cmd;
    while(1){
        vTaskDelay(pdMS_TO_TICKS(motorsController_d));

        if(state == State::IDLE){
            if(xQueueReceive(motion_cmd_q, &cmd, pdMS_TO_TICKS(0)) == pdPASS)
                startMotion(cmd);
            else{
                stopMotors();
                continue;
            }
        }
        else if(state == State::RUNNING){
            stepControl();

            if(motionFinished()){
                stopMotors();
                state = State::IDLE;
            }
        }
    }
}

void MotorsController::startMotion(MotionType_t &cmd){

    if(xQueueReceive(encoders_q, &enc_data, 10) != pdPASS)
        Serial.println("no data available from enc!");


    float theta_r = 0.0;
    float theta_l = 0.0;

    if(cmd.type == linear_dist){
        const float dist_m = cmd.value; // meters
        // wheel angle = distance / radius
        theta_r = dist_m / wheel_radius_m;
        theta_l = dist_m / wheel_radius_m;
    }
    else if(cmd.type == angular_deg){
        const float ang_rad = deg2rad(cmd.value);
        // For in-place rotation:
        // each wheel travels s = (track_width/2) * ang, each wheel rotates q = s / wheel_radius
        float s = (track_width_m * 0.5) * ang_rad;
        theta_r = s / wheel_radius_m * ROTATION_GAIN;
        theta_l = -s / wheel_radius_m * ROTATION_GAIN;
    }
    else{
        cmd.type = motion_none;
        state = State::IDLE;
        stopMotors();
        return;
    }

    trap_L.set_max_vel_accel(11, 18);
    trap_L.reset(enc_data.left, enc_data.left + theta_l);
    trap_R.set_max_vel_accel(11, 18);
    trap_R.reset(enc_data.right, enc_data.right + theta_r);

    pd_pos_R.reset();
    pd_pos_L.reset();

    state = State::RUNNING;

}

void MotorsController::stepControl(){
    // read encoders
    if(xQueueReceive(encoders_q, &enc_data, 10) != pdPASS)
        Serial.println("no data available from enc!");

    Serial.print(enc_data.left);
    Serial.print(",");
    Serial.println(enc_data.right);

    // Trapezoidal Profile
    float cmd_r = trap_R.step();
    float cmd_l = trap_L.step();

    // PID position control
    float uR = pd_pos_R.step(cmd_r, enc_data.right);
    float uL = pd_pos_L.step(cmd_l, enc_data.left);

    if(uR == 0) motor_R.stop();
    else if(uR > 0) motor_R.move(uR + 170);
    else motor_R.move(uR - 170);

    if(uL == 0) motor_L.stop();
    else if(uL > 0) motor_L.move(uL + 181);
    else motor_L.move(uL - 178);
}

bool MotorsController::motionFinished() {
    bool a = trap_L.finished() && !pd_pos_L.control_active;
    bool b = trap_R.finished() && !pd_pos_R.control_active;

    return a && b;
}

void MotorsController::stopMotors(){
    motor_R.move(0);
    motor_L.move(0);
}

float MotorsController::deg2rad(float ang){
    return ang * PI / 180;
}