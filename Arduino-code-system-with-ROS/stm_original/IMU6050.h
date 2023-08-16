#ifndef _IMU6050_
#define _IMU6050_

#include <Arduino.h>
#include "z_helper_3dmath.h"
#define INTERRUPT_PIN PA0  // use pin 2 on Arduino Uno & most boards
#define LED_PIN PC13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

class IMU6050 {
  public:
    IMU6050();
    ~IMU6050();

    //MPU6050 mpu;
    bool blinkState = false;

    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    // packet structure for InvenSense teapot demo
    uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

    // ================================================================
    // ===               INTERRUPT DETECTION ROUTINE                ===
    // ================================================================

    volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

    void intilaize();
    void dmpDataReady();
    char var;
    float *get_yaw_pitch_roll();
    bool isConnected();
    void  reset_mpu();
};

#endif
