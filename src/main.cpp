#include <Arduino.h>
#include <Servo.h>
#include "Display.hpp"
#include "IMU.hpp"
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
#define SERVO_PIN   4

InputFrame input = {};
Servo myservo;
volatile bool draw_ready = false;

/*---Profiling Variables---*/ 
unsigned long t_radio = 0;
unsigned long t_dmp =   0;

int lineht = 0;
int pos = 0;

RF24 radio(RF24_CE, SPI0_CSN);
uint8_t address[5] = { 0xCE, 0x15, 0x10, 0x55, 0xBB };
float payload = 0.0;

DisplayHelper dh = {
  Display::log_add,
  Display::log_append
};

void setup() {
  Serial1.setTX(UART0_TX);
  Serial1.setRX(UART0_RX);
  Serial1.begin(UART0_BAUD);

  SPI.setMISO(SPI0_MISO);
  SPI.setCS(SPI0_CSN);
  SPI.setSCK(SPI0_SCK);
  SPI.setMOSI(SPI0_MOSI);

  Display::init();
  Display::clear();

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
  myservo.attach(SERVO_PIN, 500, 2500);
  draw_ready = true;
}

int now = 0;
bool other_side = false;
unsigned long last_rx = micros();

void loop() {

  uint8_t pipe;
  unsigned long t0 = micros();
  if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
    uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
    radio.read(&input, bytes);             // fetch payload from FIFO
    unsigned long t1 = micros();
    unsigned long d_rx = micros() - last_rx;
    last_rx = micros();
    t_radio = t1 - t0;
    Serial1.printf("[%lu] [d%luus] RX OK\r\n", millis(), d_rx);
  }
  
  IMU::update();

  if (millis() - l_servo > 15) {
    int val = map(IMU::getAngle(), -90, 90, 500, 2500);
    myservo.write(int(val));
    now = millis();
  }
}

void loop1() {
    if (draw_ready) Display::draw(input, IMU::getAngle());
    yield();
}