//In this file are the ToF (Time of Flight) Sensor functions. The sensor is a Laser distance sensor(VL53L0X)

#pragma once
#include <Arduino.h>

#include <VL53L0X.h>


const float tof_q           = 0.3;  //Process Variance - usually a small number between 0.1 - 0.3


class ToF {
public:
    ToF();

    void init_ToF();
    bool read_ToF_mm(uint16_t& range);
    uint16_t compensateTilt(uint16_t sens);

    bool sensorInit;    //if true the sensor is initialised, else no value is reported
    uint16_t last_range = 0;
    uint16_t avr_range = 0;
    uint16_t range = 0;
    bool valid_reading = false;

private:
    VL53L0X sensor_ToF;
    uint32_t prev_millis_ToF = millis();
    bool valid_reading_prev;
    
    const float sensor_offset_x = 60.0; //mm
    const float sensor_offset_y = 0.0; //mm

    uint16_t avr_range_prev = 0;

};
