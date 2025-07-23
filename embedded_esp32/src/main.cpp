#include <Arduino.h>
#include <Main.hpp>

uint8_t motorDriverPins[] = {p_MR_A, p_MR_B, p_ML_A, p_ML_B};

SerialPublisher serialPublisher(Serial, c_1ms);
EncodersManager encodersManager(p_encoder_R, p_encoder_L, serialPublisher, c_100ms);
IMU imu(serialPublisher, c_20ms);
WheelsCon wheelController(motorDriverPins, encodersManager, imu, c_20ms);
PowerManager powerManager(serialPublisher, p_battery_level, c_2s);

SerialReceiver serialReceiver(Serial, wheelController, encodersManager, imu, c_1ms);

Task* taskList[] = { // A list of all tasks that need to be executed periodically
  &serialPublisher,
  &serialReceiver,
  &encodersManager,
  &wheelController,
  &imu,
  &powerManager,
};
TaskManager taskManager(taskList, sizeof(taskList)/sizeof(Task*));


void setup() {
  Serial.begin(115200);

  delay(100);
  serialPublisher.push_msg("I'm alive\n");
  encodersManager.initLastAngles();

  if(!imu.begin())
    serialPublisher.push_msg("MPU is not connected!\n");
}

void loop() {
  taskManager.run();
}