#include <Arduino.h>

#pragma once

#define milliTime01 2   //ms  //used for ToF.h
#define BAUD_SERIAL 115200

#define SOFT_DEBUG_RX 9
#define SOFT_DEBUG_TX 10

#define KP_PID 5
#define KI_PID 0
#define KD_PID 0

//pathControl
#define CORNER_THR 5000 //mm //If value is bigger or equal a corner is detected


//motorControl
#define MAX_PERCENT 100         //%
#define RES_MOTOR_BIT 10        
#define RES_MOTOR ((2^10) -1)   //bit //resolution of motors
