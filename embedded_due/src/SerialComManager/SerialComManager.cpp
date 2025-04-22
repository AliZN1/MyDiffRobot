#include "SerialComManager/SerialComManager.hpp"


SerialManager::SerialManager(UARTClass &b_serial):
    serial(b_serial), msg_output(numChar, QUEUE_SIZE, FIFO)
{}

SerialManager::~SerialManager(){}

void SerialManager::push_msg(const char msg[numChar]) {
    char temp[numChar];
    strncpy(temp, msg, numChar - 1);  // Copy message into a temporary array
    msg_output.push(temp);      // Push the copied message into the queue
}

void SerialManager::send_msg(){
    if(!msg_output.isEmpty()){
        char msg [numChar];
        msg_output.pop(msg);
        serial.println(msg);
    }
}
