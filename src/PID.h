#pragma once
#include <Arduino.h>
#include "struct_def.h"

#define PID_OUTPUT_CONSTRAIN_DEFAULT 0.5

class PID {

public:
    PID(float P_Gain, float D_Gain, float I_Gain, bool P_Gain_Boost, bool use_avr_on_DGain);

    void setGain(float P_Gain, float D_Gain, float I_Gain);
    float calculations(float data);

    void reset();
    float setpoint;

    float error;
private:

    void updatePrevVars(float data);
    void calcBoostP();

    

    float Kp;
    float Kd;
    float Ki;

    float PID_output;

    float data_prev;
    float error_prev;
    float integral;
    float integral_prev;
    float derivative;


    uint32_t time_prev;
    uint32_t delta_time;

    float pid_output_constrain;
    const float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
    
    bool Kp_boost_activ;
    float Kp_boost;

    bool use_avr;
    float avr_error;
};

