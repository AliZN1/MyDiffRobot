#ifndef SERIALCOM_MANAGER_H
#define SERIALCOM_MANAGER_H

#include <Arduino.h>
#include <cppQueue.h>

#define NUM_CHAR 50
#define QUEUE_SIZE 5


class SerialManager{
private:
    UARTClass &serial;
    cppQueue msg_output;
public:
    SerialManager(UARTClass &b_serial);
    ~SerialManager();
    void push_msg(const char message[NUM_CHAR]);
    void send_msg();
};

#endif