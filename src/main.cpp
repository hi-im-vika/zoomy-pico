#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <U8g2lib.h>
#include <Wire.h>

#define UART0_BAUD  115200
#define UART0_TX    0
#define UART0_RX    1
#define I2C0_SCL  21
#define I2C0_SDA  20
#define I2C1_SDA    2
#define I2C1_SCL    3
#define OLED_W      128
#define OLED_H      64
#define IMU_INT   22

U8G2_SSD1306_128X64_NONAME_F_2ND_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
MPU6050 mpu;
int const INTERRUPT_PIN = IMU_INT;  // Define the interruption #0 pin

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

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high

void DMPDataReady() {
  MPUInterrupt = true;
}

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void u8g2_draw_compass(float a) {
  // if(!a) Serial1.printf("Compass demo\n");
  Serial1.printf("%.2f\n", a);
  int line_x2 = OLED_W/2 + (30 * cos(a * M_PI/180.0));
  int line_y2 = OLED_H/2 + (30 * -sin(a * M_PI/180.0));
  u8g2.drawCircle(OLED_W/2,OLED_H/2,30);
  u8g2.drawLine(OLED_W/2,OLED_H/2,line_x2, line_y2);
}

float angle = 0;

void draw(void) {
  u8g2_prepare();
  u8g2_draw_compass(angle);
}

void setup() {
  Serial1.setTX(UART0_TX);
  Serial1.setRX(UART0_RX);
  Serial1.begin(UART0_BAUD);

  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(400000);

  Wire1.setSDA(I2C1_SDA);
  Wire1.setSCL(I2C1_SCL);
  Wire1.begin();
  
  u8g2.begin();

   /*Initialize device*/
  Serial1.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial1.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial1.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial1.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial1.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  Serial1.println(F("DMP init done"));

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(100);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(100);
    Serial1.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial1.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial1.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial1.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial1.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial1.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial1.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial1.print(devStatus);
    Serial1.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
}

void loop() {
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

  // picture loop  
  u8g2.firstPage();  
  do {
    draw();
  } while( u8g2.nextPage() );
}