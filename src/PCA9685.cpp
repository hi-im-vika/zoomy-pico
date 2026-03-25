#include "PCA9685.hpp"

namespace {
    Adafruit_PWMServoDriver pwm;
}

namespace PCA9685 {
    void init() {
        pwm = Adafruit_PWMServoDriver(0x40, Wire);
        pwm.begin();
        stopAll();
        // pwm.setPWMFreq(1000);
    }

    void setPWM(uint8_t num, uint16_t on, uint16_t off) {
        pwm.setPWM(num, on, off);
    }

    void stopAll() {
        for (int i = 0; i < 16; i++) {
            pwm.setPWM(i, 0, 4096);
        }
    }
}