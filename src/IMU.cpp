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
    
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    
    mpu6050.initialize();

      if (mpu6050.testConnection() == false) {
        return OUT_CODE_MPU_ERR; 
      }
    
    //From the reset state all registers should be 0x00, so we should be at
    //max sample rate with digital low pass filter(s) off.  All we need to
    //do is set the desired fullscale ranges
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);

    //xTaskCreatePinnedToCore(imuReadTask, "Read IMU and calculate heading", 1000, NULL, 100, &imuRead, 0);

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
    AccError.x  = AccError.x + Acc.x;
    AccError.y  = AccError.y + Acc.y;
    AccError.z  = AccError.z + Acc.z;
    GyroError.x = GyroError.x + Gyro.x;
    GyroError.y = GyroError.y + Gyro.y;
    GyroError.z = GyroError.z + Gyro.z;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccError.x  = AccError.x / c;
  AccError.y  = AccError.y / c;
  AccError.z  = AccError.z / c - 1.0;
  GyroError.x = GyroError.x / c;
  GyroError.y = GyroError.y / c;
  GyroError.z = GyroError.z / c;
  
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


float inline IMU::invSqrt(float x) {
  return 1.0/sqrtf(x);
}

void IMU::Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
  /*
   * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
   * available (for example when using the recommended MPU6050 IMU for the default setup).
   */
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    //Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    //Auxiliary variables to avoid repeated arithmetic
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //Apply feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  attitude.x = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  attitude.y = -asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  attitude.z = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
}

void IMU::imuReadTask(){
  if(micros() - current_time >= FREQUENCY_IMU){
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick6DOF(Gyro.x, -Gyro.y, -Gyro.z, -Acc.x, Acc.y, Acc.z, dt);
  }
}
