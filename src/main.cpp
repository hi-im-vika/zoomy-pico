#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define I2C0_SCL  21
#define I2C0_SDA  20
#define INT       22

MPU6050 mpu;
int const INTERRUPT_PIN = INT;  // Define the interruption #0 pin

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

void setup() {
  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(115200);

  Wire.setSDA(I2C0_SDA);
  Wire.setSCL(I2C0_SCL);
  Wire.begin();
  Wire.setClock(400000);

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
        Serial1.print("ypr\t");
        Serial1.print(ypr[0] * 180/M_PI);
        Serial1.print("\t");
        Serial1.print(ypr[1] * 180/M_PI);
        Serial1.print("\t");
        Serial1.println(ypr[2] * 180/M_PI);
    }
    MPUInterrupt = false;
  }
}