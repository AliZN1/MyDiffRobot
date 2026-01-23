#ifndef MAIN_HPP
#define MAIN_HPP

#include <cstdint>
#include <Arduino.h>
#include "SerialCom/SerialReceiver.hpp"
#include "SerialCom/SerialPublisher.hpp"
#include "Encoder/EncodersManager.hpp"
#include "Motor/MotorsManager.hpp"
#include "IMU/IMU.hpp"
#include "PowerManger/PowerManager.hpp"
#include "Config.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"


/* -------- Queues -------- */ 
// QueueHandle_t encoder_l_q;
// QueueHandle_t setpoint_l_q;
// QueueHandle_t encoder_r_q;
// QueueHandle_t setpoint_r_q;

QueueHandle_t encoders_q;
QueueHandle_t setpoints_q;
QueueHandle_t serial_out_q;
QueueHandle_t motion_cmd_q;

#endif