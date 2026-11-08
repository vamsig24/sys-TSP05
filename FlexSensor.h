#ifndef FLEX_SENSOR_H
#define FLEX_SENSOR_H

#include <Arduino.h>

class FlexSensor {
public:
    // Constructor
    // pin: Arduino analog input pin
    // fixedResistance: voltage divider resistor in ohms
    FlexSensor(uint8_t pin, float fixedResistance = 10000.0f);

    // Returns sensor resistance in kOhm
    float readResistance();

private:
    uint8_t pin;
    float fixedResistance;
};

#endif