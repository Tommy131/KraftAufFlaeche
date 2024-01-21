#include "MotorControl.h"
#include "SoftwareSerial.h"
#include "constants.h"

// This namespace is required to use Control table item names
using namespace ControlTableItem;

MotorControl::MotorControl(SoftwareSerial *_softSerial) {
    softSerial = _softSerial;
}

MotorControl::~MotorControl() {

}

void MotorControl::init(){

  //DXL Init
  dxl.begin(baudServos); 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 1; i < maxServos + 1; i++) {
      dxl.ping(i);
      dxl.reboot(i);
      dxl.torqueOff(i);
      dxl.setOperatingMode(i, operatingMode);
      dxl.torqueOn(i);
  }
      dxl.reboot(1);
      dxl.torqueOff(1);
      dxl.setOperatingMode(1, operatingMode);
      dxl.torqueOn(1);
}



void MotorControl::normalDrive(int8_t vel, int8_t rot){
    
    vel = constrain(vel, -100, 100);
    rot = constrain(rot, -100, 100);
    
    int32_t m1 = 0;
    int32_t m2 = 0;

    if (rot < 0){
        m1 = vel;
        m2 = vel  * float(rot/float(-100));

    } else if(rot > 0) {

        m2 = vel;
        m1 = vel * float(rot/float(100));

    } else {
    
        m2 = vel;
        m1 = vel;
    }
    if(m1 > 0){

    }

    bool inv_m1 = false;
    bool inv_m2 = false;
    if(m1 > 0) inv_m1 = true;
    if(m2 < 0) inv_m2 = true;
    uint32_t _m1 = map(abs(m1), 0, 100, 0, 1023);
    uint32_t _m2 = map(abs(m2), 0, 100, 0, 1023);
    
    _m1 ^= (inv_m1<<10);
    _m2 ^= (inv_m2<<10);

    dxl.setGoalVelocity(MotorMap::ML, _m1, UNIT_RAW);//-float(m1), UNIT_PERCENT);
    dxl.setGoalVelocity(MotorMap::MR, _m2, UNIT_RAW);//float(m2), UNIT_PERCENT);
    /*
    if (trim_motor < 1.0) m1 = (m1*trim_motor);
    else if (trim_motor > 1.0) m2 = (m2/trim_motor);
    */
}