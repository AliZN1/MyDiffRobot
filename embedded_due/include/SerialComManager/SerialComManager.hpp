#ifndef SERIALCOM_MANAGER_H
#define SERIALCOM_MANAGER_H

#include <Arduino.h>
#include <cppQueue.h>
#include <cstring>
#include <Config.hpp>
#include "TaskManager/Task.hpp"
#include "Wheels/WheelsCon.hpp"

#define QUEUE_SIZE 5


class SerialManager: public Task{
private:
    UARTClass &serial;
    WheelsCon &wheelsCon;
    cppQueue msg_output;
    char inputBuffer[maxNumChar];
    uint8_t inputByteIndex;
    void resetInputBuffer();
protected:
    void execute() override;
public:
    SerialManager(UARTClass &b_serial, uint16_t num_tick, WheelsCon &WheelsCon);
    ~SerialManager();
    void push_msg(const char message[maxNumChar]);
    void send_msg();
    void read_msg();
    void processMessage(const char (&msg)[maxNumChar]);
};

#endif