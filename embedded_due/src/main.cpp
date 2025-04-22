#include <Arduino.h>
#include <Main.hpp>

SerialManager serialManager(Serial, 50);
EncodersManager encodersManager(A0, A1, serialManager, 200);

Task* taskList[] = {
  &serialManager,
  &encodersManager,
};

TaskManager taskManager(taskList, sizeof(taskList)/sizeof(Task*));

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  // enc_manager.angularPos();
  taskManager.run();
}