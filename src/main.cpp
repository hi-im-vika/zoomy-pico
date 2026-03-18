#include <Arduino.h>

#define I2C0_SCL  21
#define I2C0_SDA  20
#define INT       22

void setup() {
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(115200);
}

void loop() {
}