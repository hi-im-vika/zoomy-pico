#include "Display.hpp"

namespace {
    U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C _u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
    Console _console;

    constexpr uint8_t I2C1_SDA  = 2;
    constexpr uint8_t I2C1_SCL  = 3;
    constexpr uint8_t OLED_W    = 128;
    constexpr uint8_t OLED_H    = 64;
    constexpr uint8_t OLED_CX   = OLED_W / 2;
    constexpr uint8_t OLED_CY   = OLED_H / 2;

    Cursor cursor = {0, 0};
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
        uint8_t r = 20;
        float a_rot = a + 90.0;
        uint8_t xpos = OLED_W - r - 2;
        uint8_t ypos = OLED_CY;
        int line_x2 = xpos + (r * cos(a_rot * M_PI/180.0));
        int line_y2 = ypos + (r * -sin(a_rot * M_PI/180.0));
        _u8g2.drawCircle(xpos,ypos,r);
        _u8g2.drawLine(xpos,ypos,line_x2, line_y2);
    }

    void draw_metrics(Metrics m, InputFrame i) {
        _u8g2.setFont(u8g2_font_04b_03_tr);
        _u8g2.setCursor(0,0);
        _u8g2.printf("RX total: %d", m.rx_count);
        _u8g2.setCursor(0,_u8g2.getMaxCharHeight());
        _u8g2.printf("Radio dt: %dus", m.d_radio);
        _u8g2.setCursor(0,2 * _u8g2.getMaxCharHeight());
        _u8g2.printf("Radio f: %6.0fhz", 1.0f / (m.d_radio / 1E6f));
        _u8g2.setCursor(0,3 * _u8g2.getMaxCharHeight());
        _u8g2.printf("ly/rx: %7d %7d", i.ly, i.rx);
    }

    void draw_bars(InputFrame input) {
        uint8_t max_height = OLED_CX;
        uint8_t bar_width = 4;

        uint8_t vlen1 = map(input.ly, -32767, 32767, 0, OLED_CY);
        uint8_t box1[] = { 0, OLED_H - vlen1, bar_width, vlen1};

        uint8_t vlen2 = map(input.rx, -32767, 32767, 0, OLED_CY);
        uint8_t box2[] = { bar_width + 1 , OLED_H - vlen2, bar_width, vlen2};

        _u8g2.drawBox(box1[0], box1[1], box1[2], box1[3]);
        _u8g2.drawBox(box2[0], box2[1], box2[2], box2[3]);
    }

    void draw_angle(float a) {
        char buf[10];
        sprintf(buf, "Y: %.2f", a);
        _u8g2.drawUTF8(0,0,buf);
    }

    void draw_state(State s) {
        if (!s.connected) {
            uint8_t pad = 8;
            uint8_t boxw = OLED_W - (2 * pad);
            uint8_t boxh = OLED_H - (2 * pad);
            cursor = {pad, pad};
            
            _u8g2.setDrawColor(0);
            _u8g2.drawBox(cursor.x, cursor.y, boxw, boxh);
            _u8g2.setDrawColor(1);
            _u8g2.drawFrame(cursor.x, cursor.y, boxw, boxh);
            _u8g2.setFont(u8g2_font_iconquadpix_m_all);
            cursor.x = (2 * pad) + (_u8g2.getMaxCharHeight() / 2);
            cursor.y = pad + (boxh / 2) - (_u8g2.getMaxCharHeight() / 2);
            _u8g2.drawGlyph(cursor.x, cursor.y, 33);
        }
    }

    void draw(InputFrame input, Metrics m, State s, float angle) {
        _u8g2.clearBuffer();
        draw_bars(input);
        draw_metrics(m,input);
        draw_compass(angle);
        draw_state(s);
        _u8g2.sendBuffer();
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