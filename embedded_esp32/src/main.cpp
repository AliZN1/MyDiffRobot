#include "main.hpp"


bool encoder_publisher = false;
bool imu_publisher = false;
bool power_publisher = false;
bool controller_running = false;

uint8_t motorDriverPins[] = {p_MR_A, p_MR_B, p_ML_B, p_ML_A};

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("I'm alive");
    delay(100);

    encoders_q   = xQueueCreate(1, sizeof(EncoderData_t));
    serial_out_q = xQueueCreate(SERIAL_OUT_QUEUE_LEN, sizeof(Message_t));
    motion_cmd_q = xQueueCreate(2, sizeof(MotionType_t));

    static EncodersManager encodersManager(p_encoder_R, p_encoder_L, serial_out_q, encoders_q);
    static MotorsController motorsControl(motorDriverPins, encoders_q, motion_cmd_q);
    static SerialReceiver serialRec(Serial, motion_cmd_q);
    static PowerManager powerManager(serial_out_q);
    static SerialPublisher serialPub(Serial, serial_out_q);

    xTaskCreate(Task::task, "Encoders",     2048, &encodersManager, 4, NULL);
    xTaskCreate(Task::task, "Motors",       2048, &motorsControl,   3, NULL);
    xTaskCreate(Task::task, "SerialRec",    2048, &serialRec,       1, NULL);
    xTaskCreate(Task::task, "SerialPub",    1024, &serialPub,       1, NULL);
    xTaskCreate(Task::task, "PowerManager", 2048, &powerManager,    2, NULL);
}

void loop() {}