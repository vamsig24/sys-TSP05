#ifndef FLEX_SENSOR_H
#define FLEX_SENSOR_H

#include <Arduino.h>

// Flex sensor used as the force sensor (stands in for the FSR) in a
// voltage divider:
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

    // Most recent unfiltered ADC sample (0-1023)
    int getRawADC() const;

    // Filtered sensor resistance in kOhm
    float readResistance() const;

    // Filtered sensor resistance in ohms
    float getResistanceOhms() const;

    // "Force" = bend normalised between the calibration points:
    // straight (flat) = 0 %, fully bent = 100 %
    float getForcePercent() const;

    // Endpoint-based approximation scaled to the measured maximum bend angle
    int getBendAngle() const;

    // Calibration endpoints in ohms (bent must be > flat)
    void setCalibration(float flatOhms, float bentOhms);
    float getFlatOhms() const { return flatResistance; }
    float getBentOhms() const { return bentResistance; }
    void setMaxBendDegrees(float angle) {
        if (angle >= 1.0f && angle <= 180.0f) maxBendDegrees = angle;
    }
    float getMaxBendDegrees() const { return maxBendDegrees; }

private:
    uint8_t pin;
    float fixedResistance;
    float flatResistance;
    float bentResistance;
    float maxBendDegrees = 180.0f; // must be physically measured with protractor

    // Filter
    static constexpr int MEDIAN_SIZE = 5;
    static constexpr float EMA_ALPHA = 0.15f;  // lower = smoother

    int samples[MEDIAN_SIZE] = {0};
    int sampleIdx = 0;
    int rawADC = 0;
    float smoothed = 0.0f;

    // Timing
    static constexpr unsigned long SAMPLE_TIME = 20;
    unsigned long lastSample = 0;

    int median() const;
};

#endif
