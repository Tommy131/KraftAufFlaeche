//In this file are the ToF (Time of Flight) Sensor functions. The sensor is a Laser distance sensor(VL53L0X)

#pragma once
#include <Arduino.h>

#include <VL53L0X.h>


const float tof_q           = 0.3;  //Process Variance - usually a small number between 0.1 - 0.3


class ToF {
public:

    /**
     * @brief constructor
    */
    ToF();

    /**
     * @brief init sensor and starts the continues measurement mode 
     * @return true if success, else false
    */
    bool init_ToF();

    /**
     * @brief reads the sensor in an unblocking way and uses a filter to reduce noise.
     * @param range pointer to the range object, if no valid value is read the number is not updated
     * @return true if successful read
    */
    bool read_ToF_mm(uint16_t& range);


    
    uint16_t last_range = 0;    //range from last iteration
    uint16_t avg_range = 0;     //range after the filter
    uint16_t range = 0;         //raw range value without filter 
    bool valid_reading = false; //true if a valid reading is available

private:
    bool sensorInit;            //if true the sensor is initialised, else no value is reported
    
    VL53L0X sensor_ToF;         //sensor object
    uint32_t prev_millis_ToF = millis();    
    
    bool valid_reading_prev;
    uint16_t avg_range_prev = 0;

};
