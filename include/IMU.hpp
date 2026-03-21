#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "types.hpp"

namespace IMU {
    
    void init(DisplayHelper dh);
    bool testConnection();
    uint8_t dmpInitialize();
    void calibAccel();
    void calibGyro();
    void enableDMP();
    void enableDMPInterrupt();
    float getAngle();

    void update();
}