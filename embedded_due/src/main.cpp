#include <Arduino.h>
#include <Main.hpp>

WheelsCon wheelController(2, 3, 4, 5);
SerialManager serialManager(Serial, 1, wheelController);
EncodersManager encodersManager(A1, A0, serialManager, 100);
IMU imu(serialManager, 50);

Task* taskList[] = {
  &serialManager,
  &encodersManager,
  &imu,
};
TaskManager taskManager(taskList, sizeof(taskList)/sizeof(Task*));

void setup() {
  Serial.begin(115200);

  delay(100);
  Serial.println("I'm alive!");
  encodersManager.initLastAngles();

  if(!imu.begin())
    Serial.println("MPU is not connected!");
}

void loop() {
  taskManager.run();
}