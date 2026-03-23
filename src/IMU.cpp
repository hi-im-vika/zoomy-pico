#include "IMU.hpp"

namespace {
    MPU6050 _mpu;

    constexpr uint8_t IMU_INT   = 22;

    /*---MPU6050 Control/Status Variables---*/
    bool _DMPReady = false;  // Set true if DMP init was successful
    uint8_t _devStatus;      // Return status after each device operation (0 = success, !0 = error)
    uint16_t _packetSize;    // Expected DMP packet size (default is 42 bytes)
    uint8_t _FIFOBuffer[64]; // FIFO storage buffer

    /*---Orientation/Motion Variables---*/ 
    Quaternion _q;           // [w, x, y, z]         Quaternion container
    VectorFloat _gravity;    // [x, y, z]            Gravity vector
    float _euler[3];         // [psi, theta, phi]    Euler angle container
    float _ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
    float _angle = 0.0;

    /*------Interrupt detection routine------*/
    volatile bool _interrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high

    void onInterrupt() {
        _interrupt = true;
    }
}

namespace IMU {
    void init(DisplayHelper dh) {
        pinMode(IMU_INT, INPUT);

        Serial1.println("Init MPU6050 and test connection...");
        dh.dispAdd("Init MPU...");
        _mpu.initialize();

        if(_mpu.testConnection() == false){
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
        _devStatus = _mpu.dmpInitialize();
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        Serial1.println("DMP init done");
        dh.dispAppend("OK");

        dh.dispAdd("Zero offsets...");
        _mpu.setXGyroOffset(0);
        _mpu.setYGyroOffset(0);
        _mpu.setZGyroOffset(0);
        _mpu.setXAccelOffset(0);
        _mpu.setYAccelOffset(0);
        _mpu.setZAccelOffset(0);
        dh.dispAppend("OK");

        /* Making sure it worked (returns 0 if so) */ 
        if (_devStatus == 0) {
            dh.dispAdd("Calib accel...");
            _mpu.CalibrateAccel(96);
            dh.dispAppend("OK");

            dh.dispAdd("Calib gyro...");
            _mpu.CalibrateGyro(96);
            dh.dispAppend("OK");

            _mpu.setDMPEnabled(true);
            attachInterrupt(digitalPinToInterrupt(IMU_INT), onInterrupt, RISING);

            _DMPReady = true;
        } else {
            Serial1.printf("DMP Initialization failed (code %d)", _devStatus); //Print the error code
            dh.dispAdd("DMP init fail");
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
        }
    }

    void update() {
        if (_interrupt && _DMPReady) {
            if (_mpu.dmpGetCurrentFIFOPacket(_FIFOBuffer)) {
                _mpu.dmpGetQuaternion(&_q, _FIFOBuffer);
                _mpu.dmpGetGravity(&_gravity, &_q);
                _mpu.dmpGetYawPitchRoll(_ypr, &_q, &_gravity);
                _angle = _ypr[0] * 180/M_PI;
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