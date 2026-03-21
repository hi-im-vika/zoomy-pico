#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <U8g2lib.h>
#include <Wire.h>
#include <Servo.h>
#include "Display.hpp"
#include "types.hpp"
#include <RF24.h>

#define UART0_BAUD  115200
#define UART0_TX    0
#define UART0_RX    1
#define I2C0_SCL  21
#define I2C0_SDA  20
#define SPI0_CSN    17
#define SPI0_MOSI   19
#define SPI0_MISO   16
#define SPI0_SCK    18
#define RF24_CE     27
#define RF24_INT    26
#define IMU_INT   22
#define SERVO_PIN   4

InputFrame input = {};

MPU6050 mpu;
Servo myservo;
int const INTERRUPT_PIN = IMU_INT;  // Define the interruption #0 pin
volatile bool draw_ready = false;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*---Profiling Variables---*/ 
unsigned long t_radio = 0;
unsigned long t_dmp =   0;

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high

void DMPDataReady() {
  MPUInterrupt = true;
}

float angle = 0.0;
int lineht = 0;
int pos = 0;

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
  Wire.setClock(400000);

  Display::init();
  Display::clear();

  // initialize the transceiver on the SPI bus
  Display::log_add("Init RF24...");
  if (!radio.begin()) {
    Serial1.println("radio hardware is not responding!!");
    Display::log_add("RF24 init failed");
    while (1) {}  // hold in infinite loop
  }
  Display::log_append("OK");

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setAutoAck(false);
  radio.setRetries(0,0);
  radio.setDataRate(RF24_2MBPS);
  radio.setPayloadSize(FRAME_SIZE);  // float datatype occupies 4 bytes
  radio.openReadingPipe(1, address);  // using pipe 1
  radio.startListening();

   /*Initialize device*/
  Serial1.println("Initializing I2C devices...");
  Display::log_add("Init I2C...");
  mpu.initialize();
  Display::log_append("OK");
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial1.println("Testing MPU6050 connection...");
  Display::log_add("Init MPU...");
  if(mpu.testConnection() == false){
    Serial1.println("MPU6050 connection failed");
    Display::log_add("MPU init fail");
    while(true);
  }
  else {
    Serial1.println("MPU6050 connection successful");
    Display::log_append("OK");
  }

  /* Initializate and configure the DMP*/
  Serial1.println("Initializing DMP...");
  Display::log_add("Init DMP...");
  devStatus = mpu.dmpInitialize();
  Serial1.println("DMP init done");
  Display::log_append("OK");

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    Display::log_add("Calib accel...");
    mpu.CalibrateAccel(100);  // Calibration Time: generate offsets and calibrate our MPU6050
    Display::log_append("OK");

    Display::log_add("Calib gyro...");
    mpu.CalibrateGyro(100);
    Display::log_append("OK");

    Serial1.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial1.println("Enabling DMP...");   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial1.print("Enabling interrupt detection (Arduino external interrupt ");
    Serial1.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial1.println(")...");
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial1.println("DMP ready! Waiting for first interrupt...");
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial1.print("DMP Initialization failed (code "); //Print the error code
    Serial1.print(devStatus);
    Serial1.println")");
    Display::log_add("DMP init fail");
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
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
  
  if (!DMPReady) return; // Stop the program if DMP programming fails.
    
  if (MPUInterrupt) {
    /* Read a packet from FIFO */
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
        /* Display Euler angles in degrees */
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        angle = (ypr[0] * 180/M_PI);
    }
    MPUInterrupt = false;
  }

  if (millis() - now > 15) {
    int val = map(angle, -90, 90, 500, 2500);
    myservo.write(int(val));
    now = millis();
  }
}

void loop1() {
    if (draw_ready) Display::draw(input, angle);
    yield();
}