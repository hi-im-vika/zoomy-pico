#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#define UART0_BAUD  115200
#define UART0_TX    0
#define UART0_RX    1
#define I2C0_SDA    4
#define I2C0_SCL    5
#define OLED_W      128
#define OLED_H      64

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void u8g2_draw_compass(uint8_t a) {
  // if(!a) Serial1.printf("Compass demo\n");
  u8g2.drawCircle(OLED_W/2,OLED_H/2,30);
  u8g2.drawLine(OLED_W/2,OLED_H/2,OLED_W/2 + (30 * cos(a * M_PI/180.0)), OLED_H/2 - (30 * sin(a * M_PI/180.0)));
}

uint8_t draw_state = 0;

void draw(void) {
  u8g2_prepare();
  u8g2_draw_compass(draw_state);
}

void setup() {
  Serial1.setTX(UART0_TX);
  Serial1.setRX(UART0_RX);
  Serial1.begin(UART0_BAUD);

  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  
  u8g2.begin();
}

void loop() {
  // picture loop  
  u8g2.firstPage();  
  do {
    draw();
  } while( u8g2.nextPage() );
  
  if(draw_state >= 359) {
    draw_state = 0;
  } else{
    draw_state++;
  }
}