#include <Arduino.h>

#pragma once

#define milliTime01 2   //ms  //used for ToF.h
#define BAUD_SERIAL 115200


#define KP_PID 1.0
#define KI_PID 0.0
#define KD_PID 0.0

//pathControl
#define CORNER_THR 5000 //mm //If value is bigger or equal a corner is detected


//motorControl
#define MAX_PERCENT 100         //%
#define RES_MOTOR_BIT 10        
#define RES_MOTOR ((2^10) -1)   //bit //resolution of motors

    //output codes
    const uint8_t OUT_CODE_ERR           = 0;
    const uint8_t OUT_CODE_OK            = 1;   //
    const uint8_t OUT_CODE_CORNER        = 2;   //If a corner is detected
    const uint8_t OUT_CODE_NO_TOF_MESS   = 3;   //If no ToF measurement could be done 
    const uint8_t OUT_CODE_ERR_MOTOR     = 4;   //If no ToF measurement could be done 
    const uint8_t OUT_CODE_INVAL_NUM     = 5;   //If the number Provided is invalid 

#define PIN_XSHUT_TOF_1 18 //Pin for the Xshut pin of one of the ToF sensors
#define ADDRESS_TOF_2 0x60 //Changed Address of second ToF chip

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using Arduino
    #include <SoftwareSerial.h>
    typedef SoftwareSerial SerialType;

    #define SOFT_DEBUG_RX 9
    #define SOFT_DEBUG_TX 10

#elif defined(ARDUINO_ARCH_ESP32)
    typedef HardwareSerial SerialType ;

    #define DXL_SERIAL Serial1  

    #define PIN_TX_DXL 22   //Pin for Tx communication to the Dynamixel
    #define PIN_RX_DXL 21   //Pin for Rx communication to the Dynamixel
    #define PIN_SW_DXL 16   //Pin for the switching between TX/RX communication to the Dynamixel

    #define PIN_I2C_SDA 19   //Pin for SDA communication to the Dynamixel
    #define PIN_I2C_SCL 23   //Pin for SCL communication to the Dynamixel
#endif

