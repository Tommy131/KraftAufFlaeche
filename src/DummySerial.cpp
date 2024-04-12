#include <Arduino.h>
#include "DummySerial.h"


int DummySerial::available() {
    return 0; // nothing available
}
int DummySerial::read() {
    return -1; // nothing is available
}
int DummySerial::peek() {
    return -1; // nothing available
}
size_t DummySerial::write(uint8_t c) {
    return 1; // no other option
}
