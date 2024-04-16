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

    #define ACCEL_4G
    #define GYRO_500DPS

    #define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
    #define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
    #define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
    #define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
    #define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
    #define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
    #define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
    #define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16

  
    #if defined GYRO_250DPS
    #define GYRO_SCALE GYRO_FS_SEL_250
    #define GYRO_SCALE_FACTOR 131.0
    #elif defined GYRO_500DPS
    #define GYRO_SCALE GYRO_FS_SEL_500
    #define GYRO_SCALE_FACTOR 65.5
    #elif defined GYRO_1000DPS
    #define GYRO_SCALE GYRO_FS_SEL_1000
    #define GYRO_SCALE_FACTOR 32.8
    #elif defined GYRO_2000DPS
    #define GYRO_SCALE GYRO_FS_SEL_2000
    #define GYRO_SCALE_FACTOR 16.4
    #endif

    #if defined ACCEL_2G
    #define ACCEL_SCALE ACCEL_FS_SEL_2
    #define ACCEL_SCALE_FACTOR 16384.0
    #elif defined ACCEL_4G
    #define ACCEL_SCALE ACCEL_FS_SEL_4
    #define ACCEL_SCALE_FACTOR 8192.0
    #elif defined ACCEL_8G
    #define ACCEL_SCALE ACCEL_FS_SEL_8
    #define ACCEL_SCALE_FACTOR 4096.0
    #elif defined ACCEL_16G
    #define ACCEL_SCALE ACCEL_FS_SEL_16
    #define ACCEL_SCALE_FACTOR 2048.0
    #endif


    //IMU:

    const float AccErrorX = -0.16;
    const float AccErrorY = -0.02;
    const float AccErrorZ = 0.02;
    
    const float GyroErrorX = 0.32;
    const float GyroErrorY = 0.72;
    const float GyroErrorZ = -1.73;

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
