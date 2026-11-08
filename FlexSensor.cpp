#include "FlexSensor.h"

// ====================================
// Constructor
// ====================================

FlexSensor::FlexSensor(uint8_t pin, float fixedResistance)
    : pin(pin),
      fixedResistance(fixedResistance)
{
}

// ====================================
// Read sensor resistance
// ====================================

float FlexSensor::readResistance() {

    // Read ADC (0-1023)
    int adc = analogRead(pin);

    // Avoid division by zero
    if (adc == 0) {
        return INFINITY;
    }

    // Calculate resistance using voltage divider
    // 5V -- Flex Sensor -- A0 -- 10k resistor -- GND
    float resistance =
        fixedResistance * (1023.0f / adc - 1.0f);

    // Return resistance in kOhm
    return resistance / 1000.0f;
}