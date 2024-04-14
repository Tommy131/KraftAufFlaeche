#include "IMU.h"

#include <Arduino.h>
#include <Wire.h>
#include "constants.h"
#include "MPU6050.h"
  
IMU::IMU(/* args */) {
}




outputCode IMU::init() {
  //DESCRIPTION: Initialize IMU

    #if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using Arduino
        Wire.begin();
    #elif defined(ARDUINO_ARCH_ESP32)
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    #endif
    //Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    
    mpu6050.initialize();

      if (mpu6050.testConnection() == false) {
        return OUT_CODE_MPU_ERR; 
      }
    
    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
    return OUT_CODE_OK;
}


void IMU::getIMUdata() {
    //DESCRIPTION: Request full dataset from IMU and LP filter gyro, accelerometer, and magnetometer data
    /*
    * Reads accelerometer, gyro, and magnetometer data from IMU as Acc.x, Acc.y, Acc.z, Gyro.x, Gyro.y, Gyro.z, MagX, MagY, MagZ. 
    * These values are scaled according to the IMU datasheet to put them into correct units of g's and deg/sec. A simple first-order
    * low-pass filter is used to get rid of high frequency noise in these raw signals. Generally you want to cut
    * off everything past 80Hz, but if your loop rate is not fast enough, the low pass filter will cause a lag in
    * the readings. Finally, the constant errors found in calculate_IMU_error() on startup are subtracted from the accelerometer and gyro readings.
    */
    int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;

    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);


    //Accelerometer
    Acc.x = AcX / ACCEL_SCALE_FACTOR; //G's
    Acc.y = AcY / ACCEL_SCALE_FACTOR;
    Acc.z = AcZ / ACCEL_SCALE_FACTOR;
    //Correct the outputs with the calculated error values
    Acc.x = Acc.x - AccErrorX;
    Acc.y = Acc.y - AccErrorY;
    Acc.z = Acc.z - AccErrorZ;
    //LP filter accelerometer data
    Acc.x = (1.0 - B_accel)*Acc_prev.x + B_accel*Acc.x;
    Acc.y = (1.0 - B_accel)*Acc_prev.y + B_accel*Acc.y;
    Acc.z = (1.0 - B_accel)*Acc_prev.z + B_accel*Acc.z;
    Acc_prev.x = Acc.x;
    Acc_prev.y = Acc.y;
    Acc_prev.z = Acc.z;

    //Gyro
    Gyro.x = GyX / GYRO_SCALE_FACTOR; //deg/sec
    Gyro.y = GyY / GYRO_SCALE_FACTOR;
    Gyro.z = GyZ / GYRO_SCALE_FACTOR;
    //Correct the outputs with the calculated error values
    Gyro.x = Gyro.x - GyroErrorX;
    Gyro.y = Gyro.y - GyroErrorY;
    Gyro.z = Gyro.z - GyroErrorZ;
    //LP filter gyro data
    Gyro.x = (1.0 - B_gyro)*Gyro_prev.x + B_gyro*Gyro.x;
    Gyro.y = (1.0 - B_gyro)*Gyro_prev.y + B_gyro*Gyro.y;
    Gyro.z = (1.0 - B_gyro)*Gyro_prev.z + B_gyro*Gyro.z;
    Gyro_prev.x = Gyro.x;
    Gyro_prev.y = Gyro.y;
    Gyro_prev.z = Gyro.z;

}

void IMU::calculate_IMU_error() {
  //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
  /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values Acc.x, Acc.y, Acc.z, Gyro.x, Gyro.y, Gyro.z in getIMUdata(). This eliminates drift in the
   * measurement. 
   */
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ;
  data3D AccError = {};
  data3D GyroError = {};
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    
    Acc.x  = AcX / ACCEL_SCALE_FACTOR;
    Acc.y  = AcY / ACCEL_SCALE_FACTOR;
    Acc.z  = AcZ / ACCEL_SCALE_FACTOR;
    Gyro.x = GyX / GYRO_SCALE_FACTOR;
    Gyro.y = GyY / GYRO_SCALE_FACTOR;
    Gyro.z = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccError.x  = AccErrorX + Acc.x;
    AccError.y  = AccErrorY + Acc.y;
    AccError.z  = AccErrorZ + Acc.z;
    GyroError.x = GyroErrorX + Gyro.x;
    GyroError.y = GyroErrorY + Gyro.y;
    GyroError.z = GyroErrorZ + Gyro.z;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccError.x  = AccErrorX / c;
  AccError.y  = AccErrorY / c;
  AccError.z  = AccErrorZ / c - 1.0;
  GyroError.x = GyroErrorX / c;
  GyroError.y = GyroErrorY / c;
  GyroError.z = GyroErrorZ / c;
  
  Serial.print("float AccErrorX = ");
  Serial.print(AccError.x);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccError.y);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccError.z);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroError.x);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroError.y);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroError.z);
  Serial.println(";");

}