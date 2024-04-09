#pragma once
#include <Arduino.h>
#include "PidData.h"

#define PID_OUTPUT_CONSTRAIN_DEFAULT 100.0

namespace pid {


class PID {

public:
    PID(pid_trim_t& trim, bool P_Gain_Boost, bool use_avg_on_DGain);

    void setGain(float P_Gain, float D_Gain, float I_Gain);
    void setGain(pid_trim_t& trim);
    void printGain(pid_trim_t& trim);

    float calculations(float data);

    void reset();

    /**
     * set setpoint (distance to wall)
     * @param _setpoint unit from pathControl
    */
    void setSetPoint(float setpoint);

    float error;
private:

    void updatePrevVars(float data);
    void calcBoostP();

    
    float setpoint;
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

    bool use_avg;
    float avr_error;
};

} // namespace pid
