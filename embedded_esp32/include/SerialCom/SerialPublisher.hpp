#ifndef SERIAL_PUBLISHER_HPP
#define SERIAL_PUBLISHER_HPP

#include <Arduino.h>
#include "Task.hpp"
#include "Config.hpp"
#include "freertos/queue.h"


// class HardwareSerial;

class SerialPublisher: public Task{
private:
    HardwareSerial &serial;
    QueueHandle_t serial_out_q;
    void runTask() override;

public:
    SerialPublisher(HardwareSerial &b_serialQueueHandle_t, QueueHandle_t &msg_out_q);
    ~SerialPublisher();
    void send_msg();
};

#endif