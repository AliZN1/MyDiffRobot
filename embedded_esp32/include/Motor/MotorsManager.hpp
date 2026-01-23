#pragma once

#include "stdint.h"
#include "Config.hpp"
#include "Motor/Motor.hpp"
#include "Task.hpp"
#include "freertos/queue.h"

class MotorsManager: public Task{
private:
    Motor motor_R;
    Motor motor_L;
    QueueHandle_t encoders_q;
    QueueHandle_t setpoints_q;
    void runTask() override;

public:
    MotorsManager(uint8_t (&motor_driver_pins)[4], QueueHandle_t &encoders_data_q, QueueHandle_t &motor_setpoints_q);
    ~MotorsManager();
    // void rotate();
    static float deg2rad(float ang);
};