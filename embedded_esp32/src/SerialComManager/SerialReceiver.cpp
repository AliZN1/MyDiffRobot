#include "SerialCom/SerialReceiver.hpp"


/**
 * @short SerialReceiver class constructor
 * 
 * This class manages the received data from through serial communication.
 * 
 * @param[in] b_serial Instance of Serial class which is responsible for 
 *      managing hardware for serial communication
 * @param[out] motion_command_q A reference to the queue that hold the 
 *      the input motion commands to be applied by the control unit.
 */
SerialReceiver::SerialReceiver(HardwareSerial &b_serial, QueueHandle_t &motion_command_q)
    : serial(b_serial), motion_cmd_q(motion_command_q) {}

SerialReceiver::~SerialReceiver(){ }

/**
 * Virtual override function to be run by task manager.
 *
 */
void SerialReceiver::runTask(){
    while(1){
        read_msg();

        vTaskDelay(pdMS_TO_TICKS(serialReceiver_d));
    }
}

/**
 * Clears out input buffer for serial fresh read.
 *
 */
void SerialReceiver::resetInputBuffer(){
    memset(inputBuffer, 0, MAX_MSG_IN_LEN);
    inputByteIndex = 0;
}

/**
 * @brief Reads a char from serial port.
 * 
 * If the read character is `new_line_char` the received message saved in inputBuffer will be process.
 *      Otherwise it will be added to the inputBuffer.
 *
 */
void SerialReceiver::read_msg(){
    char incomingByte;

    while(serial.available() > 0){
        incomingByte = serial.read();

        if(incomingByte == '\n'){
            processMessage(inputBuffer);    
            resetInputBuffer();
        }
        else if (inputByteIndex < MAX_MSG_IN_LEN - 1)
            inputBuffer[inputByteIndex++] = incomingByte;
        else{
            resetInputBuffer();
            serial.readString();
            Serial.println("max cmd length is 20 char!");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(serialReadByte_d));
    }
}

/**
 * @brief send motion commands to the control unit queue.
 * 
 * @param[in] type Type of the requested motion.
 * @param[in] value Value associated with requested motion.
 */
void SerialReceiver::sendMotionCmd(uint8_t type, float value){
    MotionType_t cmd;
    cmd.type = type;
    cmd.value = value;

    if(xQueueSendToBack(motion_cmd_q, &cmd, pdMS_TO_TICKS(0)) != pdPASS)
        Serial.println("Fail to update motion cmd!");
}

/**
 * @brief Processes input message to conduct any requested command through serial communication.
 *
 * If message has the correct structure of #`cmc_code`:`cmd_value`; it will be processed according the defined commands.
 * 
 * @param[in] msg An array of chars with max size of 100.
 */
void SerialReceiver::processMessage(const char (&msg)[MAX_MSG_IN_LEN]){
    if(msg[0] != '#' || msg[3] != ':'){
        Serial.println("# or : was lost!");// debuging
        return;
    }

    int cmd = 0;
    float value = 0;
    int8_t res = sscanf(msg, "#%2d:%f;", &cmd, &value); // update value with input cmd
    
    if(res != 2){
        Serial.println("Command or values caused an error!"); // debuging
        return;
    }

    switch (cmd){
        case move_dis:
            sendMotionCmd(linear_dist, value);
            break;
        case move_speed:
            sendMotionCmd(linear_speed, value);
            break;
        case rotate_deg:
            sendMotionCmd(angular_deg, value);
            break;
        case encoder_pub:
            if(value == 0) encoder_publisher = false;
            else if(value == 1) encoder_publisher = true;
            break;
        case imu_pub:
            if(value == 0) imu_publisher = false;
            else if(value == 1) imu_publisher = true;
            break;
        default:
            Serial.println("Unknown command!");
            break;
    }
}