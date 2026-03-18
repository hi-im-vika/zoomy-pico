#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void u8g2_box_title(uint8_t a) {
  if(!a) Serial1.printf("Box title demo\n");
  u8g2.drawStr( 10+a*2, 5, "U8g2");
  u8g2.drawStr( 10, 20, "GraphicsTest");
  
  u8g2.drawFrame(0,0,u8g2.getDisplayWidth(),u8g2.getDisplayHeight() );
}

void u8g2_disc_circle(uint8_t a) {
  if(!a) Serial1.printf("Disc/circle demo\n");
  u8g2.drawStr( 0, 0, "drawDisc");
  u8g2.drawDisc(10,18,9);
  u8g2.drawDisc(24+a,16,7);
  u8g2.drawStr( 0, 30, "drawCircle");
  u8g2.drawCircle(10,18+30,9);
  u8g2.drawCircle(24+a,16+30,7);
}

uint8_t draw_state = 0;

void draw(void) {
  u8g2_prepare();
  switch(draw_state >> 3) {
    case 0: u8g2_box_title(draw_state&7); break;
    case 1: u8g2_disc_circle(draw_state&7); break;
  }
}

void setup() {
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(115200);

  Wire.setSDA(4);
  Wire.setSCL(5);
  Wire.begin();
  
  u8g2.begin();
}

void loop() {
}