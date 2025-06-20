#include <Arduino.h>
#include <Main.hpp>

uint8_t motorDriverPins[] = {2, 3, 4, 5}; // {m1_A, m1_B, m2_A, m2_B}

SerialPublisher serialPublisher(Serial, 1);
EncodersManager encodersManager(A1, A0, serialPublisher, 100);
IMU imu(serialPublisher, 20);
WheelsCon wheelController(motorDriverPins, encodersManager, imu, 20);

SerialReceiver serialReceiver(Serial, 1, wheelController, encodersManager, imu);

Task* taskList[] = { // A list of all tasks that need to be executed periodically
  &serialPublisher,
  &serialReceiver,
  &encodersManager,
  &wheelController,
  &imu,
};
TaskManager taskManager(taskList, sizeof(taskList)/sizeof(Task*));

uint32_t last_time = 0;

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