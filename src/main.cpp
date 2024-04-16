#include "pinDef.h"
#include "constants.h"
#include <DynamixelShield.h>
#include "PID.h"
#include "ToF.h"
#include "MotorControl.h"
#include "pathControl.h"
#include "PidData.h"
#include "RuntimeConfig.h"
#include "DummySerial.h"

#if defined(PIO_UNIT_TESTING)
  DummySerial serial_out;
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using Arduino
  //Init for SoftSerial
  #include <SoftwareSerial.h>
  SoftwareSerial sw_serial_out(SOFT_DEBUG_RX, SOFT_DEBUG_TX); // DYNAMIXELShield UART RX/TX
  #define serial_out (sw_serial_out)
#elif defined(ARDUINO_ARCH_ESP32)
  #include <DNSServer.h>
  HardwareSerial& serial_out = Serial;
#endif


//Init for PID
pid::PID pidWall(default_pid_trim, 0, false, serial_out);
pid::PID pidAngle(default_pid_angle_trim, 0, false, serial_out);

//Init ToF
ToF backWall(BACK_TOF_VALUE_OFFSET);
ToF frontWall(0);

pathControl path(DEFAULT_DISTANCE, serial_out, nullptr, &frontWall, &backWall, &pidWall, &pidAngle);

#if defined(RUNTIME_CONFIG_ENABLE) && defined(ARDUINO_ARCH_ESP32)
DNSServer dns;
runtimeconfig::RuntimeConfig runtimeConfig(serial_out, dns);
#endif

#ifndef PIO_UNIT_TESTING // for unit testing
void setup() {

  serial_out.begin(BAUD_SERIAL);
  serial_out.println("Setting up...");
  
  pinMode(PIN_XSHUT_TOF_1, OUTPUT);
  digitalWrite(PIN_XSHUT_TOF_1, HIGH);

  #if defined(RUNTIME_CONFIG_ENABLE) && defined(ARDUINO_ARCH_ESP32)
  runtimeConfig.setOnGainDistUpdateCallback([](pid::pid_trim_t& upd) {
    pidWall.setGain(upd);
    pidWall.printGain(upd);
  });

  runtimeConfig.setOnGainAngleUpdateCallback([](pid::pid_trim_t& upd) {
    pidAngle.setGain(upd);
    pidAngle.printGain(upd);
  });


  runtimeConfig.setOnSpeedUpdate([](int8_t speed) {
  path.setSpeed(speed);
  });

  runtimeConfig.setOnDistanceUpdateCallback([](uint16_t distance) {
    path.setDist(distance);
  });

  runtimeConfig.setupRuntimeConfig();
  #endif

#if defined(ARDUINO_ARCH_ESP32)

  serial_out.printf("PathInitCode(1): %d\n", path.init());
  while (touchRead(PIN_TOUCH) >= TOUCH_THR); //Wait for start Signal

#else
  serial_out.print("PathInitCode(1): ");
  serial_out.println(path.init());
#endif

  serial_out.println("Now entering main loop()");
  
} // setup

// uint32_t loop_time = millis();
// uint32_t loop_delta = 0;

void loop() {

  path.loop();
  
  #if defined(RUNTIME_CONFIG_ENABLE) && defined(ARDUINO_ARCH_ESP32)
    runtimeConfig.loopRuntimeConfig();
  #endif

  // serial_out.printf("loopDelta: %dms\n", loop_delta);
  // if(millis()-loop_time > loop_delta) loop_delta = millis()-loop_time;
  // if((millis()%1000) == 0) loop_delta = 0;
  // loop_time = millis();

} // loop
#endif // PIO_UNIT_TESTING


/**
 * Possible Start/Stop config with touch sensor
 * Global:
 *
// bool stopLoop = false;
// uint16_t touchCNT = 0;
 *
 * In loop():
// if(touchRead(PIN_TOUCH) <= TOUCH_THR) touchCNT ++;
// else                                  touchCNT = 0;
// if(touchCNT >= 5000) {
//   stopLoop = !stopLoop;
//   touchCNT = 0;
//   while(touchRead(PIN_TOUCH) <= TOUCH_THR);
// }
// if(!stopLoop){
  path.loop();
// } else {
//   //path.stopMotors();
// }
// runtimeConfig.loopRuntimeConfig();
*/
