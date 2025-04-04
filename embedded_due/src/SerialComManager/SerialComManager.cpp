#include "SerialComManager/SerialComManager.hpp"


SerialManager::SerialManager(UARTClass &b_serial):
    serial(b_serial), msg_output(NUM_CHAR, QUEUE_SIZE, FIFO)
{}

SerialManager::~SerialManager(){}

void SerialManager::push_msg(const char msg[NUM_CHAR]) {
    char temp[NUM_CHAR];
    strncpy(temp, msg, NUM_CHAR - 1);  // Copy message into a temporary array
    temp[NUM_CHAR - 1] = '\0';  // Ensure null termination
    msg_output.push(temp);      // Push the copied message into the queue
}

void SerialManager::send_msg(){
    if(!msg_output.isEmpty()){
        char msg [NUM_CHAR];
        msg_output.pop(msg);
        serial.println(msg);
    }
}
