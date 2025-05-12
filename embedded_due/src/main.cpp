#include <Arduino.h>
#include <Main.hpp>

WheelsCon wheelController(2, 3, 4, 5);
SerialManager serialManager(Serial, 1, wheelController);
EncodersManager encodersManager(A0, A1, serialManager, 500);

Task* taskList[] = {
  &serialManager,
  // &encodersManager,
};
TaskManager taskManager(taskList, sizeof(taskList)/sizeof(Task*));

void setup() {
  Serial.begin(115200);
  encodersManager.initLastAngles();
  
  delay(100);
  Serial.println("I'm alive!");
}

void loop() {
  taskManager.run();
}