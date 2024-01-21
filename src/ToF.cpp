#include "ToF.h"

#include <Arduino.h>
#include <constants.h>
#include <Wire.h>

TwoWire Wire1;

ToF::ToF() {
}



void ToF::init_ToF(){
    Wire1.begin();
    Wire1.setClock(1000000);    //note that it is way bigger than the max from the datasheet(400kHz) 
    sensorInit = sensor_ToF.init();
    if(!sensorInit) return;
    sensor_ToF.startContinuous();
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
            uint16_t temp_range = 4642;
            temp_range = sensor_ToF.readRangeContinuousMillimeters();
            if(temp_range == 4642)      valid_reading = valid_reading_prev;  //indicates no data has been changed
            else if(temp_range >= 3000) valid_reading = false; //indicates generally a bad reading(max range 2000mm)
            else {
                //Serial.print("tempRange: "); Serial.print(temp_range); Serial.print(" avrRange_bev:"); Serial.print(avr_range);
                //avr_range = (temp_range + avr_range)/2;
                avr_range = ((float)avr_range_prev + tof_q * ((float)temp_range - (float)avr_range_prev)); //filter the values
                avr_range_prev = avr_range;
                last_range = temp_range;
                valid_reading = true;
                //Serial.print(" avr_range_after: "); Serial.print(avr_range); Serial.println(" avr!");
                range_in = avr_range;
            }
        }
    }
    valid_reading_prev = valid_reading;
    return valid_reading;
}

