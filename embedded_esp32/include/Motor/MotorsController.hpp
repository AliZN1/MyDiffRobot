#pragma once

#include "stdint.h"
#include "Config.hpp"
#include "Motor/Motor.hpp"
#include "Control/PID.hpp"
#include "Control/TrapProfile.hpp"
#include "Task.hpp"
#include "freertos/queue.h"

#define pos_PID_Kp           10
#define pos_PID_Ki           0
#define pos_PID_Kd           0.4
#define pos_PID_Kff          0 // feed forward gain
#define time_constant        0.01
#define motor_saturation_max 254
#define motor_saturation_min -254
#define wheelRad_axleLen     0.53719 // wheel radius over axle length
#define v_lin_max_mps        2.0 // m/s
#define a_lin_max_mps2       1.0 // m/s^2
#define w_ang_max_rps        4.0 // rad/s
#define alpha_max_rps2       8.0 // rad/s^2


enum class State : uint8_t {
    IDLE = 0,
    RUNNING = 1
};

class MotorsController: public Task{
private:
    Motor motor_R;
    Motor motor_L;
    PID pid_pos_R;
    PID pid_pos_L;
    TrapProfile prof_R;
    TrapProfile prof_L;
    QueueHandle_t encoders_q;
    QueueHandle_t motion_cmd_q;
    State state;
    EncoderData_t enc_data;
    void startMotion(MotionType_t &cmd);
    void stepControl(float dt);
    bool motionFinished();
    void stopMotors();
    void runTask() override;
    static float deg2rad(float ang);

public:
    MotorsController(uint8_t (&motor_driver_pins)[4], QueueHandle_t &encoders_data_q, QueueHandle_t &motion_q);
    ~MotorsController();
};