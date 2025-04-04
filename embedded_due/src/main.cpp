#include <Arduino.h>
#include <Main.hpp>

SerialManager serialManager(Serial);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  // enc_manager.angularPos();
  delay(100);
  serialManager.push_msg("1000");
  delay(10);
  serialManager.send_msg();
}