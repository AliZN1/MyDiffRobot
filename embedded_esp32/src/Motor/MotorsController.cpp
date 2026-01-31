#include "Motor/MotorsController.hpp"


MotorsController::MotorsController(uint8_t (&mdp)[4], QueueHandle_t &encoders_data_q, QueueHandle_t &motion_q)
    : motor_R(mdp[0], mdp[1]), 
    motor_L(mdp[3], mdp[2]), 
    pid_pos_R(pos_PID_Kp, pos_PID_Ki, pos_PID_Kd, pos_PID_Kff, time_constant),
    pid_pos_L(pos_PID_Kp, pos_PID_Ki, pos_PID_Kd, pos_PID_Kff, time_constant),
    encoders_q(encoders_data_q), 
    motion_cmd_q(motion_q)
{
    pid_pos_R.set_saturation(motor_saturation_max, motor_saturation_min);
    pid_pos_L.set_saturation(motor_saturation_max, motor_saturation_min);
    enc_data.right = 0.0;
    enc_data.left = 0.0;
}

MotorsController::~MotorsController() {}

void MotorsController::runTask(){
    state = State::IDLE;

    // Initialize to current position
    if(xQueueReceive(encoders_q, &enc_data, 10) == pdPASS)
        Serial.println("no data available from enc!");

    prof_R.reset(enc_data.right, enc_data.right);
    prof_L.reset(enc_data.left, enc_data.left);

    pid_pos_R.reset();
    pid_pos_L.reset();

    const float dt = motorsController_d * 0.001f;
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
            stepControl(dt);

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


    float dR_rad = 0.0;
    float dL_rad = 0.0;

    if(cmd.type == linear_dist){
        const float dist_m = cmd.value; // meters
        // wheel angle = distance / radius
        dR_rad = dist_m / wheel_radius_m;
        dL_rad = dist_m / wheel_radius_m;

        // Limits in wheel space
        prof_R.set_max_vel_accel(v_lin_max_mps / wheel_radius_m, a_lin_max_mps2 / wheel_radius_m);
        prof_L.set_max_vel_accel(v_lin_max_mps / wheel_radius_m, a_lin_max_mps2 / wheel_radius_m);
    }
    else if(cmd.type == angular_deg){
        const float ang_rad = deg2rad(cmd.value);
        // For in-place rotation:
        // each wheel travels s = (track_width/2) * ang, each wheel rotates q = s / wheel_radius
        float s = (track_width_m * 0.5) * ang_rad;
        dR_rad = s / wheel_radius_m;
        dL_rad = -s / wheel_radius_m;

        // Limits in wheel space derived from max robot angular velocity:
        // wheel linear speed = (track_width/2) * w
        // wheel angular speed = v_wheel / r
        float v_wheel_max_mps  = (track_width_m * 0.5f) * w_ang_max_rps;
        float a_wheel_max_mps2 = (track_width_m * 0.5f) * alpha_max_rps2;

        prof_R.set_max_vel_accel(v_wheel_max_mps / wheel_radius_m, a_wheel_max_mps2 / wheel_radius_m);
        prof_L.set_max_vel_accel(v_wheel_max_mps / wheel_radius_m, a_wheel_max_mps2 / wheel_radius_m);
    }
    else{
        cmd.type = motion_none;
        state = State::IDLE;
        stopMotors();
        return;
    }

    prof_R.reset(enc_data.right, enc_data.right + dR_rad);
    prof_L.reset(enc_data.left, enc_data.left + dL_rad);

    pid_pos_R.reset();
    pid_pos_L.reset();

    state = State::RUNNING;

}

void MotorsController::stepControl(float dt){
    // read encoders
    if(xQueueReceive(encoders_q, &enc_data, 10) != pdPASS)
        Serial.println("no data available from enc!");

    // update position setpoints from the profiles
    const float qR_sp = prof_R.step(dt);
    const float qL_sp = prof_L.step(dt);

    // PID position control
    float uR = pid_pos_R.step(qR_sp, enc_data.right);
    float uL = pid_pos_L.step(qL_sp, enc_data.left);

    // Optional: deadband / minimum drive to overcome stiction
    // uR = applyDeadband(uR); uL = applyDeadband(uL);

    if(uR == 0) motor_R.stop();
    else if(uR > 0) motor_R.move(uR + 190);
    else motor_R.move(uR - 190);

    if(uL == 0) motor_L.stop();
    else if(uL > 0) motor_L.move(uL + 190);
    else motor_L.move(uL - 190);
}

bool MotorsController::motionFinished() {
    // We check profile completion (position+velocity).
    // You can also check encoder error directly if you want.
    return prof_R.finished() && prof_L.finished();
}

void MotorsController::stopMotors(){
    motor_R.move(0);
    motor_L.move(0);
}

float MotorsController::deg2rad(float ang){
    return ang * PI / 180;
}