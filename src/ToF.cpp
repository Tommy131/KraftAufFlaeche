#include "ToF.h"

#include <Arduino.h>
#include <constants.h>
#include <Wire.h>


ToF::ToF() {
}


bool ToF::init_ToF(){
    Wire.begin();
    Wire.setClock(400000);
    sensorInit = sensor_ToF.init();
    if(!sensorInit) return false;
    sensor_ToF.startContinuous();
    return true;
}

uint16_t ToF::read_ToF_mm(){
    uint16_t range_in = 0;
    if(read_ToF_mm(range_in)) return range_in;
    return last_range;
}

/**
 * reads the distance information in an nonBlocking way. if no valid value is found, range is not updated.
 * @param range uint mm, is the returned distance from the ground
 * @return returns true if value is valid false otherwise
 */
bool ToF::read_ToF_mm(uint16_t& range_in){
    if(millis() - prev_millis_ToF >= milliTime01){
        prev_millis_ToF = millis();
        if(!sensorInit)                 valid_reading = false; //do nothing if not init
        else {
            uint16_t temp_range = 0;
            temp_range = sensor_ToF.readRangeContinuousMillimeters();
            if(temp_range >= 3000) valid_reading = false; //indicates generally a bad reading(max range 2000mm)
            else {
                //Serial.print("tempRange: "); Serial.print(temp_range); Serial.print(" avrRange_bev:"); Serial.print(avg_range);
                //avg_range = (temp_range + avg_range)/2;
                avg_range = ((float)avg_range_prev + tof_q * ((float)temp_range - (float)avg_range_prev)); //filter the values
                avg_range_prev = avg_range;
                last_range = temp_range;
                valid_reading = true;
                //Serial.print(" avr_range_after: "); Serial.print(avg_range); Serial.println(" avr!");
                range_in = avg_range;
            }
        }
    }
    valid_reading_prev = valid_reading;
    return valid_reading;
}
