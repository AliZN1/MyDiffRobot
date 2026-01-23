#ifndef SERIAL_RECEIVER_HPP
#define SERIAL_RECEIVER_HPP

#include <Arduino.h>
#include <cstring>
#include "Config.hpp"
#include "Task.hpp"
#include "freertos/queue.h"


// enum SensorDataType { 
//     Encoders_t, 
//     IMU_t,
//     Battery_t,
// };

class SerialReceiver: public Task{
private:
    HardwareSerial &serial;
    QueueHandle_t &motion_cmd_q;

    char inputBuffer[MAX_MSG_IN_LEN];
    uint8_t inputByteIndex;
    void resetInputBuffer();
    void runTask() override;
public:
    SerialReceiver(HardwareSerial &b_serial, QueueHandle_t &motion_command_q);
    ~SerialReceiver();
    void read_msg();
    void processMessage(const char (&msg)[MAX_MSG_IN_LEN]);
    void sendMotionCmd(uint8_t type, float value);
};

#endif