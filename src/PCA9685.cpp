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

    void motorControl(Motor m, int value) {
        int magnitude = abs(value);
        if (!value) {
            switch (m) {
                case M_NE:
                    pwm.setPWM(0,0,4096);
                    pwm.setPWM(1,0,4096);
                    break;
                case M_NW:
                    pwm.setPWM(2,0,4096);
                    pwm.setPWM(3,0,4096);
                    break;
                case M_SE:
                    pwm.setPWM(4,0,4096);
                    pwm.setPWM(5,0,4096);
                    break;
                case M_SW:
                    pwm.setPWM(6,0,4096);
                    pwm.setPWM(7,0,4096);
                    break;
                default:
                    break;
            }
            return;
        }
        if (value > 0) {
            switch (m) {
                case M_NE:
                    pwm.setPWM(0,0,magnitude);
                    pwm.setPWM(1,0,4096);
                    break;
                case M_NW:
                    pwm.setPWM(2,0,magnitude);
                    pwm.setPWM(3,0,4096);
                    break;
                case M_SE:
                    pwm.setPWM(4,0,magnitude);
                    pwm.setPWM(5,0,4096);
                    break;
                case M_SW:
                    pwm.setPWM(6,0,magnitude);
                    pwm.setPWM(7,0,4096);
                    break;
                default:
                    break;
            }
            return;
        } else {
            switch (m) {
                case M_NE:
                    pwm.setPWM(0,0,4096);
                    pwm.setPWM(1,0,magnitude);
                    break;
                case M_NW:
                    pwm.setPWM(2,0,4096);
                    pwm.setPWM(3,0,magnitude);
                    break;
                case M_SE:
                    pwm.setPWM(4,0,4096);
                    pwm.setPWM(5,0,magnitude);
                    break;
                case M_SW:
                    pwm.setPWM(6,0,4096);
                    pwm.setPWM(7,0,magnitude);
                    break;
                default:
                    break;
            }
            return;
        }
    }
}