#include "SerialComManager/SerialComManager.hpp"


SerialManager::SerialManager(UARTClass &b_serial, uint16_t num_tick, WheelsCon &WheelsControllers)
    :  Task(num_tick), serial(b_serial), wheelsCon(WheelsControllers), msg_output(maxNumChar, QUEUE_SIZE, FIFO) {}

SerialManager::~SerialManager(){}

void SerialManager::push_msg(const char msg[maxNumChar]) {
    char temp[maxNumChar];
    strncpy(temp, msg, maxNumChar - 1);  // Copy message into a temporary array
    msg_output.push(temp);      // Push the copied message into the queue
}

void SerialManager::send_msg(){
    if(!msg_output.isEmpty()){
        char msg [maxNumChar];
        msg_output.pop(msg);
        serial.println(msg);
    }
}

void SerialManager::execute(){
    send_msg();
    read_msg();
}

void SerialManager::resetInputBuffer(){
    inputBuffer[maxNumChar-1] = '\0';
    inputByteIndex = 0;
}

void SerialManager::read_msg(){
    if(serial.available() <= 0)
        return;
    char incomingByte = serial.read();
    if(incomingByte == '\n'){
        resetInputBuffer();
        processMessage(inputBuffer);
    } else{
        if (inputByteIndex < maxNumChar - 1)
            inputBuffer[inputByteIndex++] = incomingByte;
        else
            inputByteIndex = 0;
    }
}

void SerialManager::processMessage(const char (&msg)[maxNumChar]){
    if(msg[0] != '#'){
        // next two lines are use for debuging
        char warning[maxNumChar] = "Incorrect input!";
        push_msg(warning);
        
        return;
    }
    if(strstr(msg, "FW")){
        // wheelsCon.moveFW(100);
        Serial.println("Forward");
    }
    else if(strstr(msg, "BW")){
        // wheelsCon.moveBW(100);
        Serial.println("Backward");
    }
}


