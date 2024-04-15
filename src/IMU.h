#pragma once

#include <Arduino.h>
#include "MPU6050.h"
#include "constants.h"

class IMU {


public:
    IMU(/* args */);

    struct data3D {
        float x;
        float y;
        float z;
    };

    outputCode init();
    void getIMUdata();
    void calculate_IMU_error();

    data3D getAccel()    const { return Acc; }
    data3D getGyro()     const { return Gyro; }
    data3D getAttitude() const { return attitude; }

    void imuReadTask();

private:
    MPU6050 mpu6050;
    #define ACCEL_SCALE MPU6050_ACCEL_FS_2
    #define ACCEL_SCALE_FACTOR 16384.0
    #define GYRO_SCALE MPU6050_GYRO_FS_250
    #define GYRO_SCALE_FACTOR 131.0

    //IMU:

    const float AccErrorX = 1.02;
    const float AccErrorY = -0.02;
    const float AccErrorZ = -1.15;


    const float GyroErrorX = -1.71;
    const float GyroErrorY = 0.71;
    const float GyroErrorZ = 0.33;

    float B_madgwick = 0.04;  //Madgwick filter parameter
    const float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14.)
    const float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1)
    
    float q0 = 1.0f; //Initialize quaternion for madgwick filter
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    data3D Acc;
    data3D Gyro;

    data3D Acc_prev;
    data3D Gyro_prev;

    data3D attitude;

    void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
    float inline invSqrt(float x);

    uint64_t current_time = micros();
    uint64_t prev_time = micros();
    float dt;
    #define FREQUENCY_IMU 500 //us //
    
    

};
