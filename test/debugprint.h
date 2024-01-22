#pragma once

#include "unity.h"
#include "Arduino.h"

#define MAX_MSG_LEN 500

int debug_printf(const char *fmt, ...);

#define debug_printf(...) debug_printf(__VA_ARGS__)

