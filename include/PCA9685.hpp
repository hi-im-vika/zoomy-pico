#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

namespace PCA9685 {
    void init();
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    void stopAll();
}