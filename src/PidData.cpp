#include "PidData.h"

/**
 *  TODO: calibrate
 */
pid::pid_trim_t default_pid_trim = {
  .kp = 1,
  .ki = 0.0,
  .kd = 0.0,
};

pid::pid_trim_t default_pid_angle_trim = {
  .kp = -100.0,
  .ki = 0.0,
  .kd = 0.0,
};

