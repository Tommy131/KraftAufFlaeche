#include "PID.h"

namespace pid {

PID::PID(pid_trim_t& trim, bool P_Gain_Boost, bool use_avg_on_DGain)
    : error(0)
    , PID_output(0) 
    , data_prev(0)
    , error_prev(0)
    , integral(0)
    , integral_prev(0)
    , derivative(0)
    , time_prev(micros())
    , delta_time(0)
    , pid_output_constrain(PID_OUTPUT_CONSTRAIN_DEFAULT)
    , Kp_boost(0)
    , avr_error(0)
    {
    
    Kp = trim.kp;
    Ki = trim.ki;
    Kd = trim.kd;
    

    Kp_boost_activ = P_Gain_Boost;
    use_avg = use_avg_on_DGain;
}


void PID::setGain(float P_Gain, float D_Gain, float I_Gain){
    Kp = P_Gain;
    Kd = D_Gain;
    Ki = I_Gain;
}


void PID::setGain(pid_trim_t& trim) {
    Serial.println("Updating the trim");
    Kp = trim.kp;
    Ki = trim.ki;
    Kd = trim.kd;
}

void PID::printGain(pid_trim_t& trim) {
    #if defined(ARDUINO_ARCH_ESP32)
    Serial.println("Updated the configuration to:");
    Serial.printf(
        "KP: %f\n"
        "KI: %f\n"
        "KD: %f\n", trim.kp, trim.ki, trim.kd
    );
    #endif
}


void PID::calcBoostP(){
    if(error < -1 || error > 1){
    Kp_boost = (fabs(error) - 1) / 20;
    Kp_boost = constrain(Kp_boost, 0, 0.01);
    }  else Kp_boost = 0;
}

float PID::calculations(float data){
    delta_time = micros() - time_prev;

    error = setpoint - data;
    integral = integral_prev + error * delta_time;
    integral = constrain(integral, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup

    if(Kp_boost_activ) calcBoostP();
    else Kp_boost = 0;

    if(use_avg){
        avr_error = (error + avr_error) / 2;
        derivative = avr_error;
    } else derivative = (error - error_prev) / delta_time; 

    PID_output = ((Kp + Kp_boost) * error + Ki * integral + Kd * derivative);
    PID_output = constrain(PID_output, -pid_output_constrain, pid_output_constrain);
    //Update altitude vars
    error_prev = error;
    integral_prev = integral;
    data_prev = data;
    time_prev = micros();

    return PID_output;
}

void PID::reset(){
    error = error_prev = 0;
    integral = integral_prev = 0;
    PID_output = 0;
}

void PID::setSetPoint(float _setpoint) {
    setpoint = _setpoint;
}

} // namespace pid
