#include "SerialComManager/SerialComManager.hpp"


//------------------- Serial Publisher class members

SerialPublisher::SerialPublisher(UARTClass &b_serial, uint16_t num_tick)
    :  Task(num_tick), serial(b_serial), msg_output(maxNumChar, QUEUE_SIZE, FIFO) {}

SerialPublisher::~SerialPublisher(){}

void SerialPublisher::execute(){
    send_msg();
}

void SerialPublisher::push_msg(const char msg[maxNumChar]) {
    char temp[maxNumChar];
    strncpy(temp, msg, maxNumChar - 1);  // Copy message into a temporary array
    msg_output.push(temp);      // Push the copied message into the queue
}

void SerialPublisher::send_msg(){
    if(!msg_output.isEmpty()){
        char msg [maxNumChar];
        msg_output.pop(msg);
        serial.print(msg);
    }
}

//------------------- Serial Receiver class members

SerialReceiver::SerialReceiver(UARTClass &b_serial, uint16_t num_tick, WheelsCon &wheelsController, EncodersManager &encoders_manager, IMU &IMU_sensor)
    : Task(num_tick), serial(b_serial), wheelsCon(wheelsController), encodersManager(encoders_manager), imu(IMU_sensor) {}

SerialReceiver::~SerialReceiver(){ }

void SerialReceiver::execute(){
    read_msg();
}

void SerialReceiver::resetInputBuffer(){
    inputBuffer[maxNumChar-1] = '\0';
    inputByteIndex = 0;
}

void SerialReceiver::read_msg(){
    if(serial.available() <= 0) return;
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

void SerialReceiver::processMessage(const char (&msg)[maxNumChar]){
    if(msg[0] != '#' || msg[3] != ':'){
        // Serial.println("# or : was lost!");// debuging
        return;
    }

    int cmd = 0;
    int value = 0;
    int8_t res = sscanf(msg, "#%2d:%d;", &cmd, &value); // update value with input cmd
    
    if(res != 2){
        // Serial.println("Command or value caused error!"); // debuging
        return;
    }

    switch (cmd){
        case move_speed:
            if(value == 0) wheelsCon.stop();
            else wheelsCon.set_linearVel(value);
            break;
        case rotate_speed:
            wheelsCon.set_angularPos(value);
            break;
        case encoder_pub:
            if(value == 0) encodersManager.set_publish(0);
            else if(value == 1) encodersManager.set_publish(1);
            break;
        case imu_pub:
            if(value == 0) imu.set_publish(0);
            else if(value == 1) imu.set_publish(1);
            break;
        default:
            break;
    }
}