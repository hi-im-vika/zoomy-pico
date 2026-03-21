#include "IMU.hpp"

namespace {
    MPU6050 _mpu;

    constexpr uint8_t I2C0_SDA  = 20;
    constexpr uint8_t I2C0_SCL  = 21;
    constexpr uint8_t IMU_INT   = 22;

    DisplayHelper _dh;

    /*---MPU6050 Control/Status Variables---*/
    bool DMPReady = false;  // Set true if DMP init was successful
    uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
    uint8_t FIFOBuffer[64]; // FIFO storage buffer

    /*---Orientation/Motion Variables---*/ 
    Quaternion q;           // [w, x, y, z]         Quaternion container
    VectorFloat gravity;    // [x, y, z]            Gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
    float _angle = 0.0;

    /*------Interrupt detection routine------*/
    volatile bool _interrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high

    void onInterrupt() {
        _interrupt = true;
    }
}

namespace IMU {
    void init(DisplayHelper dh) {

        _dh = dh;

        Wire.setSDA(I2C0_SDA);
        Wire.setSCL(I2C0_SCL);
        Wire.begin();
        Wire.setClock(400000);

        pinMode(IMU_INT, INPUT);

        Serial1.println("Init MPU6050 and test connection...");
        dh.dispAdd("Init MPU...");
        _mpu.initialize();

        if(IMU::testConnection() == false){
            Serial1.println("MPU6050 connection failed");
            dh.dispAdd("MPU init fail");
            while(true);
        } else {
            Serial1.println("MPU6050 connection successful");
            dh.dispAppend("OK");
        }

        /* Initializate and configure the DMP*/
        Serial1.println("Initializing DMP...");
        dh.dispAdd("Init DMP...");
        uint8_t devStatus = IMU::dmpInitialize();
        Serial1.println("DMP init done");
        dh.dispAppend("OK");

        /* Making sure it worked (returns 0 if so) */ 
        if (devStatus == 0) {
            dh.dispAdd("Calib accel...");
            IMU::calibAccel();
            dh.dispAppend("OK");

            dh.dispAdd("Calib gyro...");
            IMU::calibGyro();
            dh.dispAppend("OK");

            IMU::enableDMP();
            IMU::enableDMPInterrupt();
        } else {
            Serial1.printf("DMP Initialization failed (code %d)", devStatus); //Print the error code
            dh.dispAdd("DMP init fail");
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
        }
    }

    bool testConnection() {
        return _mpu.testConnection();
    }

    uint8_t dmpInitialize() {
        return _mpu.dmpInitialize();
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
    }

    void zeroOffsets() {
        /* Supply your gyro offsets here, scaled for min sensitivity */
        _mpu.setXGyroOffset(0);
        _mpu.setYGyroOffset(0);
        _mpu.setZGyroOffset(0);
        _mpu.setXAccelOffset(0);
        _mpu.setYAccelOffset(0);
        _mpu.setZAccelOffset(0);
    }

    void calibAccel() {
        _mpu.CalibrateAccel(100);
    }

    void calibGyro() {
        _mpu.CalibrateGyro(100);
    }

    void enableDMP() {
        _mpu.setDMPEnabled(true);
    }

    void enableDMPInterrupt() {
        attachInterrupt(digitalPinToInterrupt(IMU_INT), onInterrupt, RISING);
    }

    uint16_t getDMPFIFOPacketSize() {
        return _mpu.dmpGetFIFOPacketSize();
    }

    void update() {
        if (_interrupt) {
            if (_mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
                _mpu.dmpGetQuaternion(&q, FIFOBuffer);
                _mpu.dmpGetGravity(&gravity, &q);
                _mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                _angle = ypr[0] * 180/M_PI;
            } else {
                _mpu.resetFIFO();
            }
            _interrupt = false;
        }
    }

    float getAngle() {
        return _angle;
    }
}