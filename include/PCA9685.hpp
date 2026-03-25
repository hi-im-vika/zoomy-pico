#pragma once

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

namespace PCA9685 {
    enum Motor {
        M_NE,
        M_NW,
        M_SE,
        M_SW
    };

    void init();
    void setPWM(uint8_t num, uint16_t on, uint16_t off);
    void stopAll();

    void motorControl(Motor m, int value);
}