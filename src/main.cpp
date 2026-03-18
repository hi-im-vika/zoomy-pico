#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

void setup() {
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(115200);
}

void loop() {
}