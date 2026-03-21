#include "Display.hpp"

namespace {
    U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C _u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
    Console _console;
}

namespace Display {
    void init() {
        Wire1.setSDA(I2C1_SDA);
        Wire1.setSCL(I2C1_SCL);
        Wire1.begin();
        _u8g2.begin();
        prepare();

        _console.init(&_u8g2);
    }

    void prepare() {
        _u8g2.setFont(u8g2_font_6x10_tf);
        _u8g2.setFontRefHeightExtendedText();
        _u8g2.setDrawColor(1);
        _u8g2.setFontPosTop();
        _u8g2.setFontDirection(0);
    }

    void draw_compass(float a) {
        float a_rot = a + 90.0;
        int line_x2 = OLED_W/2 + (30 * cos(a_rot * M_PI/180.0));
        int line_y2 = OLED_H/2 + (30 * -sin(a_rot * M_PI/180.0));
        _u8g2.drawCircle(OLED_W/2,OLED_H/2,30);
        _u8g2.drawLine(OLED_W/2,OLED_H/2,line_x2, line_y2);
    }

    void draw_angle(float a) {
        char buf[10];
        sprintf(buf, "Y: %.2f", a);
        _u8g2.drawUTF8(0,0,buf);
    }

    void draw_rx(int16_t a) {
        char buf[10];
        sprintf(buf, "RX: %d", a);
        _u8g2.drawUTF8(0,10,buf);
    }

    void draw(InputFrame input, float angle) {
        // Display::prepare();
        _u8g2.clearBuffer();					// clear the internal memory
        draw_angle(angle);
        draw_rx(input.lx);
        draw_compass(angle);
        _u8g2.sendBuffer();					// transfer internal memory to the display
    }

    void clear() {
        _u8g2.clearBuffer();
    }

    void log_add(char *str) {
        _console.add(str);
        _console.display();
    }

    void log_append(char *str) {
        _console.append(str);
        _console.display();
    }
}