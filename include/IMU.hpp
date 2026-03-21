#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "types.hpp"

namespace IMU {
    
    void init(DisplayHelper dh);
    void update();
    float getAngle();
}