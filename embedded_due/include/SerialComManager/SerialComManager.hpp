#ifndef SERIALCOM_MANAGER_H
#define SERIALCOM_MANAGER_H

#include <Arduino.h>
#include <cppQueue.h>
#include <Config.hpp>
#include "TaskManager/Task.hpp"

#define QUEUE_SIZE 5


class SerialManager: public Task{
private:
    UARTClass &serial;
    cppQueue msg_output;
protected:
    void execute() override;
public:
    SerialManager(UARTClass &b_serial, uint16_t num_tick);
    ~SerialManager();
    void push_msg(const char message[numChar]);
    void send_msg();
};

#endif