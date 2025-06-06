#include <Arduino.h>
#include <Main.hpp>

uint8_t motorDriverPins[] = {2, 3, 4, 5}; // {m1_A, m1_B, m2_A, m2_B}

SerialPublisher serialPublisher(Serial, 1);
EncodersManager encodersManager(A1, A0, serialPublisher, 100);
IMU imu(serialPublisher, 20);
WheelsCon wheelController(motorDriverPins, encodersManager, imu, 20);

SerialReceiver serialReceiver(Serial, 1, wheelController, encodersManager, imu);

Task* taskList[] = {
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
  Serial.println("I'm alive!");
  encodersManager.initLastAngles();

  if(!imu.begin())
    Serial.println("MPU is not connected!");
}

void loop() {
  taskManager.run();
  
  // int pause = 20;

  // analogWrite(2, 200);
  // analogWrite(3, 0);
  // analogWrite(4, 0);
  // analogWrite(5, 200);
  
  // delay(int(pause/2));
  
  // analogWrite(2, 50);
  // analogWrite(3, 0);
  // analogWrite(4, 0);
  // analogWrite(5, 50);

  // delay(pause);
}