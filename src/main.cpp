#include <Arduino.h>

void setup() {
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(115200);
}

void loop() {
}