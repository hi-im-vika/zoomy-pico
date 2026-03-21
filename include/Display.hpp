#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "Console.hpp"
#include "types.hpp"

#define I2C1_SDA    2
#define I2C1_SCL    3
#define OLED_W      128
#define OLED_H      64

namespace Display {
    
    void init();
    void prepare();
    void draw_compass(float a);
    void draw_angle(float a);
    void draw_rx(int16_t a);
    void draw(InputFrame input, float angle);
    void clear();

    void log_add(char *str);
    void log_append(char *str);
}