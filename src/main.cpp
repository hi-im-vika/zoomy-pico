#include <Arduino.h>
#include <Servo.h>
#include "Display.hpp"
#include "IMU.hpp"
#include "PCA9685.hpp"
#include "types.hpp"
#include <RF24.h>

#define UART0_BAUD  115200
#define UART0_TX    0
#define UART0_RX    1
#define SPI0_CSN    17
#define SPI0_MOSI   19
#define SPI0_MISO   16
#define SPI0_SCK    18
#define RF24_CE     27
#define RF24_INT    26
#define STEERING_PIN 4
#define THROTTLE_PIN 5
#define STR_MAX     2000
#define STR_MIN     1000
#define THT_MAX     2500
#define THT_MIN     500

constexpr uint8_t I2C0_SDA  = 20;
constexpr uint8_t I2C0_SCL  = 21;

InputFrame input = {};
Metrics draw_metrics = {};
State draw_state = {};
Servo steering, throttle;
volatile bool draw_ready = false;
bool connected = false;

/*---Profiling Variables---*/ 
unsigned long rx_count = 0;

unsigned long t_radio = 0;

unsigned long l_servo = millis();
unsigned long l_radio = micros();

unsigned long d_radio = 0;

RF24 radio(RF24_CE, SPI0_CSN);
uint8_t address[5] = { 0xCE, 0x15, 0x10, 0x55, 0xBB };
float payload = 0.0;

void setup() {
  Serial1.setTX(UART0_TX);
  Serial1.setRX(UART0_RX);
  Serial1.begin(UART0_BAUD);

  SPI.setMISO(SPI0_MISO);
  SPI.setCS(SPI0_CSN);
  SPI.setSCK(SPI0_SCK);
  SPI.setMOSI(SPI0_MOSI);

  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(1000000);

  Display::init();
  Display::clear();

  Display::log_add("Init PCA9685...");
  PCA9685::init();
  Display::log_append("OK");

  // initialize the transceiver on the SPI bus
  Display::log_add("Init RF24...");
  if (!radio.begin()) {
    Serial1.println("radio hardware is not responding!!");
    Display::log_add("RF24 init failed");
    while(true);
  }
  Display::log_append("OK");

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setAutoAck(false);
  radio.setRetries(0,0);
  radio.setDataRate(RF24_2MBPS);
  radio.setPayloadSize(FRAME_SIZE);  // float datatype occupies 4 bytes
  radio.openReadingPipe(1, address);  // using pipe 1
  radio.startListening();

  IMU::init(DisplayHelper{
    Display::log_add,
    Display::log_append
  });

  Display::log_add("All OK!");
  delay(1000);
  Display::clear();
  steering.attach(STEERING_PIN, STR_MIN, STR_MAX);
  throttle.attach(THROTTLE_PIN, THT_MIN, THT_MAX);
  draw_ready = true;
}

void loop() {

  uint8_t pipe;
  if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
    unsigned long now = micros();
    d_radio = now - l_radio;
    l_radio = micros();
    uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
    radio.read(&input, bytes);             // fetch payload from FIFO
    rx_count++;
  }

  State s = {
    micros() - l_radio < 500000,
    l_radio
  };

  draw_state = s;
  
  IMU::update();

  if (!draw_state.connected) {
    int throttle_raw = map(input.ly, -32767, 32767, STR_MIN, STR_MAX);
    int steering_raw = map(input.rx, -32767, 32767, THT_MIN, THT_MAX);
    throttle.write(throttle_raw);
    steering.write(steering_raw);
    l_servo = millis();
  }

  if (micros() - l_radio > 500000) {
    // failsafe mode activated
    rx_count = d_radio = 0;
    input = {0,0,0,0,0};
  }

  Metrics m = {
    rx_count, d_radio
  };
  draw_metrics = m;
  draw_ready = true;
}

void loop1() {
    if (draw_ready) {
      // draw_ready = false;
      Metrics m = draw_metrics;
      State s = draw_state;
      Display::draw(input, m, s, IMU::getAngle());
    }
    yield();
}