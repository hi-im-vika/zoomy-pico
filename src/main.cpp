#include <Arduino.h>

#define I2C1_SCL  27
#define I2C1_SDA  26
#define INT       22

void setup() {
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(115200);
}

void loop() {
}