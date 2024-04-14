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

    data3D getAccel()   const { return Acc; }
    data3D getGyro()    const { return Gyro; }

private:
    MPU6050 mpu6050;
    #define ACCEL_SCALE MPU6050_ACCEL_FS_2
    #define ACCEL_SCALE_FACTOR 16384.0
    #define GYRO_SCALE MPU6050_GYRO_FS_250
    #define GYRO_SCALE_FACTOR 131.0

    //IMU:
    float AccX, AccY, AccZ;
    float AccX_prev, AccY_prev, AccZ_prev;
    float GyroX, GyroY, GyroZ;
    float GyroX_prev, GyroY_prev, GyroZ_prev;
    float q0 = 1.0f; //Initialize quaternion for madgwick filter
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;


    const float AccErrorX = 0.00;
    const float AccErrorY = -0.01;
    const float AccErrorZ = -0.14;


    const float GyroErrorX = 1.02;
    const float GyroErrorY = 0.64;
    const float GyroErrorZ = -0.97;

    const float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    const float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    
    data3D Acc;
    data3D Gyro;

    data3D Acc_prev;
    data3D Gyro_prev;

};
