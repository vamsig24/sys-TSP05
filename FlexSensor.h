#ifndef FLEX_SENSOR_H
#define FLEX_SENSOR_H

#include <Arduino.h>

// Flex sensor in a voltage divider:
// 5V -- flex sensor -- pin -- fixedResistance -- GND
// Bend with the printed side on the outside of the curve.
//
// Non-blocking: call update() from loop(). Each sample is median-filtered
// (last MEDIAN_SIZE readings) and then smoothed with an EMA.
class FlexSensor {
public:
    // pin: Arduino analog input pin
    // fixedResistance: voltage divider resistor in ohms
    // flatResistance / bentResistance: calibration in ohms (straight / max bend)
    FlexSensor(uint8_t pin,
               float fixedResistance = 10000.0f,
               float flatResistance = 13000.0f,
               float bentResistance = 34000.0f);

    // Fills the filter with initial readings
    void begin();

    // Call repeatedly from loop(); samples every SAMPLE_TIME ms
    void update();

    // Filtered ADC value (0-1023)
    float getADC() const;

    // Filtered sensor resistance in kOhm
    float readResistance() const;

    // Bend mapped from resistance: straight = 0, max bend = 180 degrees
    int getBendAngle() const;

private:
    uint8_t pin;
    float fixedResistance;
    float flatResistance;
    float bentResistance;

    // Filter
    static constexpr int MEDIAN_SIZE = 5;
    static constexpr float EMA_ALPHA = 0.15f;  // lower = smoother

    int samples[MEDIAN_SIZE] = {0};
    int sampleIdx = 0;
    float smoothed = 0.0f;

    // Timing
    static constexpr unsigned long SAMPLE_TIME = 20;
    unsigned long lastSample = 0;

    int median() const;
    float resistanceOhms() const;
};

#endif
