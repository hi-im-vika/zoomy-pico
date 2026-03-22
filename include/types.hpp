#pragma once
#include <Arduino.h>

#define FRAME_SIZE 10

#pragma pack(push, 1)
struct InputFrame {
  int16_t  lx, ly, rx, ry;
  uint16_t buttons;
};
#pragma pack(pop)
static_assert(sizeof(InputFrame) == FRAME_SIZE, "frame size mismatch");

struct DisplayHelper {
  void (*dispAdd)(char *str);
  void (*dispAppend)(char *str);
};

struct Metrics {
  unsigned long rx_count;
  unsigned long d_radio;
};