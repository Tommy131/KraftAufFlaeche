#pragma once
#include <Arduino.h>

/**
 * @brief This class should implement the interface for the print functions
*/

class DummySerial: public Stream
{
public:
    int available();
    int read();
    int peek();
    size_t write(uint8_t c);
};
