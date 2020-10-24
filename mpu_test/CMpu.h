#pragma once

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class CMpu {

public: 
    MPU6050* mpu = new MPU6050;

    void init();
    void processMpu();
    void dmpDataReady();

    float mpuYaw = 0;
    float mpuPitch = 0;
    float mpuRoll = 0;

    volatile bool mpuInterrupt = false;

private:
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful

    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container

    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector

    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
};;