#include "main.hpp"


bool encoder_publisher = false;
bool imu_publisher = false;
bool power_publisher = false;

uint8_t motorDriverPins[] = {p_MR_A, p_MR_B, p_ML_B, p_ML_A};

void readSerial(void *parameter){
    char msg[32];
    int8_t idx = 0;

    while(1){
        if(!Serial.available()){
            vTaskDelay(150);
            continue;
        }

        char incomingByte = Serial.read();
        if(incomingByte == '\n'){
            msg[idx] = '\0'; // end the string
            
            // process the msg
            MotorSetpoint_t setpoint;
            setpoint.left = atoff(msg);
            setpoint.right = setpoint.left;

            if(xQueueSendToBack(setpoints_q, &setpoint, (TickType_t)10) != pdPASS)
                Serial.println("Queue is full!");

            // reset the buffer
            msg[0] = '\0';
            idx = 0;
        }
        if(idx < 32-1)
            msg[idx++] = incomingByte; // store the char
        else{
            // reset the buffer
            msg[0] = '\0';
            idx = 0;
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("I'm alive");
    delay(100);

    encoders_q   = xQueueCreate(2, sizeof(EncoderData_t));
    setpoints_q  = xQueueCreate(1, sizeof(MotorSetpoint_t));
    serial_out_q = xQueueCreate(SERIAL_OUT_QUEUE_LEN, sizeof(Message_t));
    motion_cmd_q = xQueueCreate(2, sizeof(MotionType_t));

    static EncodersManager encodersManager(p_encoder_R, p_encoder_L, serial_out_q, encoders_q);
    static MotorsManager motorsManager(motorDriverPins, encoders_q, setpoints_q);

    xTaskCreate(Task::task, "Encoders", 2048, &encodersManager, 2, NULL);
    xTaskCreate(Task::task, "Motors", 2048, &motorsManager, 1, NULL);
    xTaskCreate(readSerial, "Serial", 2048, NULL, 1, NULL);
}

void loop() {}