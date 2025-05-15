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
        serial.print(msg);
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
    if(msg[0] != '#' || msg[3] != ':'){
        // next two lines are use for debuging
        // char warning[maxNumChar] = "# or : was lost!";
        // push_msg(warning);

        return;
    }
    char command[10] = {0};
    int value = 0;
    int res = sscanf(msg, "#%[^:]:%d;", command, &value);
    
    if(res != 2){
        // next two lines are use for debuging
        // char warning[maxNumChar] = "Command or value caused error!";
        // push_msg(warning);   
        return;
    }

    if(strcmp(command, "MS") == 0){ // Move with this speed
        if(value == 0)
            wheelsCon.stop();
        else if(value > 0 && value < 255)
            wheelsCon.moveFW(value);
        else if(value < 0 && value > -255)
            wheelsCon.moveBW(-value);
    }
    else if(strcmp(command, "RS") == 0){ // Rotate with this speed
        if(value == 0)
            wheelsCon.stop();
        else if(value > 0 && value < 255)
            wheelsCon.rotateCW(value);
        else if(value < 0 && value > -255)
            wheelsCon.rotateCCW(-value);
    }
}


