#include "PidData.h"

pid::pid_trim_t default_pid_trim = {
  .kp = 0.5,
  .ki = 0.0,
  .kd = 0.0,
};

pid::pid_trim_t default_pid_angle_trim = {
  .kp = -100.0,
  .ki = 0.0,
  .kd = 0.0,
};

