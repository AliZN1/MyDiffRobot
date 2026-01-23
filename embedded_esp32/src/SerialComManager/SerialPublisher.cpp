#include "SerialCom/SerialPublisher.hpp"


//------------------- Serial Publisher class members

/**
 * @short SerialReceiver class constructor
 * 
 * This class manages the received data from through serial communication.
 * 
 * @param[out] b_serial Instance of Serial class which is responsible for 
 *      managing hardware for serial communication
 * @param[in] msg_out_q Reference to message queue handler. This is the queue that 
 *      serial communication message are written into.
 */
SerialPublisher::SerialPublisher(HardwareSerial &b_serial, QueueHandle_t &msg_out_q)
    : serial(b_serial), serial_out_q(msg_out_q) {}

SerialPublisher::~SerialPublisher(){}

void SerialPublisher::runTask(){
    send_msg();
}

/**
 * Adds a new message to be sent by Serial Port.
 *
 * @param[in] msg An array of chars with maximum size of 100.
 */
// void SerialPublisher::push_msg(const char msg[maxNumChar]) {
//     char temp[maxNumChar];
//     strncpy(temp, msg, maxNumChar - 1);  // Copy message into a temporary array
//     msg_output.push(temp);      // Push the copied message into the queue
// }

/**
 * Serial prints the earliest message has been added to the queue and pop it out.
 *
 */
void SerialPublisher::send_msg(){
    Message_t temp_bf;

    if(xQueueReceive(serial_out_q, &temp_bf, 0) != pdPASS) 
        return;

    serial.print(temp_bf.text);
}