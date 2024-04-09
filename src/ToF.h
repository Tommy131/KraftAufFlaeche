//In this file are the ToF (Time of Flight) Sensor functions. The sensor is a Laser distance sensor(VL53L0X)

#pragma once
#include <Arduino.h>
#include <VL53L0X.h>


#include "constants.h"


const float tof_q           = 0.8;  //Process Variance - usually a small number between 0.1 - 0.3


class ToF {
public:

    /**
     * @brief constructor
    */
    ToF(float _correction);

    /**
     * @brief init sensor and starts the continues measurement mode 
     * @param PIN_OFF if set defines the Pin with that the other ToF sensor can be disabled
     * @param changeAdress this is the new address with that the sensor should be addressed
     * @return OUT_CODE_ERR and OUT_CODE_OK, OUT_CODE_INVAL_NUM if changeAddress is the same as default
    */
    uint8_t init_ToF(int8_t PIN_OFF  = -1, uint8_t changeAdress  = ADDRESS_TOF_1);

    /**
     * @brief reads the sensor in an unblocking way and uses a filter to reduce noise.
     * @param range pointer to the range object, if no valid value is read the number is not updated
     * @return true if successful read
    */
    bool read_ToF_mm(uint16_t& range);

    uint16_t read_ToF_mm();

    bool getInit() const        { return sensorInit; }
    

    uint16_t getLastRange() const   { return last_range; }
    uint16_t getAvgRange()  const   { return avg_range; }
    bool     getValidRead() const   { return valid_reading; }


private:
    bool sensorInit;            //if true the sensor is initialised, else no value is reported
    
    VL53L0X sensor_ToF;         //sensor object
    uint32_t prev_millis_ToF = millis();    
    
    bool valid_reading_prev;
    uint16_t avg_range_prev = 0;

    uint16_t last_range = 0;    //range from last iteration without filter 
    uint16_t avg_range = 0;     //range after the filter
    bool valid_reading = false; //true if a valid reading is available

    float correction;
};
