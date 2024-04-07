#pragma once
namespace pid {

/**
 * Struct describing the trim
*/
typedef struct {
  float kp;
  float ki;
  float kd;
} pid_trim_t;

}

extern pid::pid_trim_t default_pid_trim;

