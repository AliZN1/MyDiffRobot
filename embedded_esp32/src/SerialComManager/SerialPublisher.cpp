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
    while(1){
        send_msg();

        vTaskDelay(pdMS_TO_TICKS(serialPublisher_d));
    }
}

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