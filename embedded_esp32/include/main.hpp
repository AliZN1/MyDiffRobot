#ifndef MAIN_HPP
#define MAIN_HPP

#include <cstdint>
#include <Arduino.h>
#include "SerialCom/SerialReceiver.hpp"
#include "SerialCom/SerialPublisher.hpp"
#include "Encoder/EncodersManager.hpp"
#include "Motor/MotorsController.hpp"
#include "IMU/IMU.hpp"
#include "PowerManger/PowerManager.hpp"
#include "Config.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/stream_buffer.h"


/* -------- Queues -------- */ 
QueueHandle_t encoders_q;
QueueHandle_t serial_out_q;
QueueHandle_t motion_cmd_q;

#endif