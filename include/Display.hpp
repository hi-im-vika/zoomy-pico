#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "Console.hpp"
#include "types.hpp"

namespace Display {
    
    void init();
    void prepare();
    void draw_compass(float a);
    void draw_angle(float a);
    void draw(InputFrame input, Metrics m, State s, float angle);
    void draw_state(State s);
    void clear();

    void log_add(char *str);
    void log_append(char *str);
}