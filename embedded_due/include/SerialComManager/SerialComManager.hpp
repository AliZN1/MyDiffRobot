#ifndef SERIALCOM_MANAGER_H
#define SERIALCOM_MANAGER_H

#include <Arduino.h>
#include <cppQueue.h>
#include <Config.hpp>

#define QUEUE_SIZE 5


class SerialManager{
private:
    UARTClass &serial;
    cppQueue msg_output;
public:
    SerialManager(UARTClass &b_serial);
    ~SerialManager();
    void push_msg(const char message[numChar]);
    void send_msg();
};

#endif