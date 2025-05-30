#include <Arduino.h>
#include <Main.hpp>

uint8_t motorDriverPins[] = {2, 3, 4, 5}; // {m1_A, m1_B, m2_A, m2_B}

SerialManager serialManager(Serial, 1);
EncodersManager encodersManager(A1, A0, serialManager, 100);
WheelsCon wheelController(motorDriverPins, encodersManager, 100);
// IMU imu(serialManager, 50);

Task* taskList[] = {
  &serialManager,
  &encodersManager,
  &wheelController,
  // &imu,
};
TaskManager taskManager(taskList, sizeof(taskList)/sizeof(Task*));

uint16_t i = 7;
uint32_t last_time = millis();

void setup() {
  Serial.begin(115200);

  delay(100);
  Serial.println("I'm alive!");
  encodersManager.initLastAngles();

  // if(!imu.begin())
  //   Serial.println("MPU is not connected!");

  // wheelController.moveFW(5);
}

void loop() {
  taskManager.run();

  uint32_t now = millis();
  if(now > last_time + 10000){
    if(i > 15){
      wheelController.stop();
      return;
    }

    wheelController.moveFW(i);
    Serial.println(i);
    i += 2;
    last_time = now;
  }
}