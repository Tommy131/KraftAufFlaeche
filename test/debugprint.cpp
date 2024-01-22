#include <Arduino.h>
#include "unity.h"
#include "debugprint.h"

int debug_printf(const char *fmt,  ...) {
    int n;
    char msg[MAX_MSG_LEN];
    memset(msg, 0, MAX_MSG_LEN);

    va_list args;
    va_start(args, fmt);

    n = vsnprintf(msg, MAX_MSG_LEN-1, fmt, args);
    TEST_MESSAGE(msg);
    va_end(args);
    return n;
}
