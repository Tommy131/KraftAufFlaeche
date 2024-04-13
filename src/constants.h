#include <Arduino.h>
#include "DummySerial.h"

#pragma once

#define milliTime01 2   //ms  //used for ToF.h
#define BAUD_SERIAL 115200


//pathControl
#define CORNER_THR 500 //mm //If value is bigger or equal a corner is detected
#define MAX_SPEED 100   //PERCENT that the controller should aim to meet
#define DEFAULT_DISTANCE 100

#define DISTANCE_TOF 78.0 //mm //distance between the two sensors
#define DISTANCE_TOF_MID 40 //mm //distance between rotation point(mid plane of Robot) and sensor
#define BACK_TOF_VALUE_OFFSET -9 //mm //measurement offset of this sensor  
#define ANGLE_TO_WALL_PARALLEL (90 * DEG_TO_RAD) //RAD //angle the robot has if it is parallel to the wall

#ifndef DEG_TO_RAD
    #define DEG_TO_RAD     = (PI/180);
#endif //DEG_TO_RAD
#ifndef RAD_TO_DEG
    #define RAD_TO_DEG     = (1/(PI/180));
#endif //RAD_TO_DEG


//motorControl
#define MAX_PERCENT 100         //%
#define RES_MOTOR_BIT 10        
#define RES_MOTOR (1023)   //bit //resolution of motors

//output codes
enum outputCode{
    OUT_CODE_ERR           = 0,
    OUT_CODE_OK            = 1,   
    OUT_CODE_CORNER        = 2,   //If a corner is detected
    OUT_CODE_NO_TOF_MESS   = 3,   //If no ToF measurement could be done 
    OUT_CODE_ERR_MOTOR     = 4,   //When problems motors are 
    OUT_CODE_INVAL_NUM     = 5,   //If the number Provided is invalid 
    OUT_CODE_PASS          = 6,   //If no ERR or OK state could be determent 
};
    

#define PIN_XSHUT_TOF_1 18 //Pin for the Xshut pin of one of the ToF sensors
#define ADDRESS_TOF_1 0x29 //Default Address of second ToF chip
#define ADDRESS_TOF_2 0x60 //Changed Address of second ToF chip

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using Arduino
    #define SOFT_DEBUG_RX 9
    #define SOFT_DEBUG_TX 10

#elif defined(ARDUINO_ARCH_ESP32)
    #define PIN_TX_DXL 22   // Pin for Tx communication to the Dynamixel
    #define PIN_RX_DXL 21   // Pin for Rx communication to the Dynamixel
    #define PIN_SW_DXL 16   // Pin for the switching between TX/RX
                            // communication to the Dynamixel (DXL)
                            // HIGH -> send to DXL
                            // LOW  -> receive from DXL

    #define PIN_I2C_SDA 19  // Pin for SDA communication to the sensor(s)
    #define PIN_I2C_SCL 23  // Pin for SCL communication to the sensor(s)

    #define PIN_TOUCH   27  // Pin for Capacitive Sensor
    #define TOUCH_THR   60  //Threshold for the Capacitive Pin of ESP32

#endif

#if defined(PIO_UNIT_TESTING)
    typedef DummySerial SerialType;
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using Arduino
    #include <SoftwareSerial.h>
    typedef SoftwareSerial SerialType;
#elif defined(ARDUINO_ARCH_ESP32)
    typedef HardwareSerial SerialType;
    #define DXL_SERIAL Serial1
#endif

#define RUNTIME_CONFIG_ENABLE 1
