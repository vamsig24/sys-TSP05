#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include <Arduino.h>

// Potentiometer as a voltage divider on an analog pin.
// Non-blocking: call update() from loop(). Readings are smoothed with a
// moving average over the last SAMPLE_COUNT samples.
class Potentiometer {
public:
    // pin: Arduino analog input pin
    // adcReference: ADC reference voltage (nominal 5 V on the Uno)
    Potentiometer(uint8_t pin, float adcReference = 5.0f);

    // Fills the moving average buffer with initial readings
    void begin();

    // Call repeatedly from loop(); samples every SAMPLE_TIME ms
    void update();

    // Most recent raw ADC value (0-1023)
    int getRawADC() const;

    // Moving average ADC value (0-1023)
    float getFilteredADC() const;

    // Filtered wiper voltage
    float getVoltage() const;

    // Filtered position, 0-100 %
    int getPositionPercent() const;
    // Nominal mechanical sweep, measured against a protractor for each part.
    // Edit POT_TRAVEL_DEG after measuring this specific potentiometer.
    float getAngleDegrees() const;


private:
    uint8_t pin;
    float adcReference;
    static constexpr float POT_TRAVEL_DEG = 270.0f;

    // Moving average
    static constexpr int SAMPLE_COUNT = 10;
    int samples[SAMPLE_COUNT] = {0};
    int sampleIndex = 0;
    long total = 0;

    int rawADC = 0;

    // Timing
    static constexpr unsigned long SAMPLE_TIME = 100;
    unsigned long lastSample = 0;
};

#endif
