#define UNITY_INCLUDE_PRINT_FORMATTED

#include "unity.h"
#include "MotorControl.h"
#include "../debugprint.h"
#include "ToF.h"
#include "DummySerial.h"


void setUp(void) {}


void tearDown(void) {}


/* Logic as of meeting from 21.01.24

m1 = 100% -> 2047
m2 = 100% -> 1023

m1 = 70% -> 1740
m2 = 70% -> 716

m1 = -70% -> 716
m2 = -70% -> 1740

m1 = 30% -> 1330
m2 = 30%-> 306

m1 = -30% -> 306
m2 = -30%-> 1330

m1 = 0% -> 0
m2 = 0% -> 0
*/

typedef struct motor_conv {
    int16_t in;
    uint16_t out;
} motor_conv_t;


typedef struct converstions {
    motor_conv_t m1;
    motor_conv_t m2;

} conversion_t;


conversion_t test_conversions[] = {
    {
        .m1 = { .in = 100, .out = 2047 },
        .m2 = { .in = 100, .out = 1023 },
    },
    {
        .m1 = { .in = 70, .out = 1740 },
        .m2 = { .in = 70, .out = 716 },
    },
    {
        .m1 = { .in = -70, .out = 716 },
        .m2 = { .in = -70, .out = 1740 },
    },
    {
        .m1 = { .in = 30, .out = 1330 },
        .m2 = { .in = 30, .out =  306 },
    },
    {
        .m1 = { .in = -30, .out = 306 },
        .m2 = { .in = -30, .out =  1330 },
    },
    {
        .m1 = { .in = 0, .out = 0 },
        .m2 = { .in = 0, .out =  0 },
    },
};

#ifdef PIO_UNIT_TESTING

void test_motor_calc() {
    DummySerial dummy_serial;
    MotorControl motorctrl(dummy_serial);
    
    for (size_t i = 0; i < (sizeof(test_conversions) / sizeof(conversion_t)); i++) {
        conversion_t conv = test_conversions[i];
        const uint16_t m1 = motorctrl.calc_motor_vel(conv.m1.in, true);
        debug_printf("m1 velocity %" PRId16 " -> % " PRIu16 " (expect %" PRIu16 ")", 
                            conv.m1.in, m1, conv.m1.out);
        TEST_ASSERT_EQUAL(conv.m1.out, m1);

        const uint16_t m2 = motorctrl.calc_motor_vel(conv.m1.in, false);
        debug_printf("m2 velocity %" PRId16 " -> % " PRIu16 " (expect %" PRIu16 ")", 
                            conv.m2.in, m2, conv.m2.out);
        TEST_ASSERT_EQUAL(conv.m2.out, m2);
    }

}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_motor_calc);
    UNITY_END();
}

void loop() {}
#endif
