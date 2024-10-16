#include "ToF.h"

#include <Arduino.h>
#include <constants.h>
#include <Wire.h>


ToF::ToF(float _correction) {
    offset = _correction;
}


outputCode ToF::init_ToF(int8_t pin_off /* = -1*/, uint8_t changeAddress /* = 0x29*/){
    sensorInit = false;

    #if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using Arduino
    Wire.begin();
    #elif defined(ARDUINO_ARCH_ESP32)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    #endif


    if(pin_off != -1) {
        if(changeAddress == ADDRESS_TOF_1) return OUT_CODE_INVAL_NUM;
        digitalWrite(pin_off, LOW);

        sensorInit = sensor_ToF.init();
        if(!sensorInit){
            sensor_ToF.setAddress(changeAddress);
        }
    }
    //Wire.setClock(400000);
    if(!sensorInit) sensorInit = sensor_ToF.init();

    if(!sensorInit) {
        if(pin_off != -1) digitalWrite(pin_off, HIGH);
        return OUT_CODE_ERR;
    }
    if(pin_off != -1) {
        sensor_ToF.setAddress(changeAddress);
        digitalWrite(pin_off, HIGH);
    }
    sensor_ToF.setMeasurementTimingBudget(updateToF);   //us
    Wire.setClock(1000000);    //note that it is way bigger than the max from the datasheet(400kHz)
    sensor_ToF.startContinuous();
    return OUT_CODE_OK;
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
            temp_range = sensor_ToF.readRangeContinuousMillimeters() + offset;
            if(temp_range >= MAX_READING) valid_reading = false; //indicates generally a bad reading(max range 2000mm)
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

/**
 * 130 -> 117
 * 80 -> 71
*/
