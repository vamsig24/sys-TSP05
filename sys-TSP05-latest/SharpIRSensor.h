#ifndef SHARP_IR_SENSOR_H
#define SHARP_IR_SENSOR_H

#include <Arduino.h>

// Sharp GP2Y0A21YK0F analog IR distance sensor (model "1080", 10-80 cm).
// Non-blocking: call update() from loop(). Raw electrical ADC stays unfiltered
// for transfer plots. Distance uses median-of-25 ADC + spike confirmation + EMA.
class SharpIRSensor {
public:
    enum Status {
        INVALID_READING,
        TOO_CLOSE,
        TOO_FAR,
        VALID
    };

    // pin: Arduino analog input pin
    // minDistanceCm / maxDistanceCm: valid measuring range of the sensor
    SharpIRSensor(uint8_t pin,
                  int minDistanceCm = 10,
                  int maxDistanceCm = 80);

    void begin();

    // Call repeatedly from loop(); takes a new reading every SAMPLE_TIME ms
    void update();

    // Most recent distance in cm (may be outside the valid range)
    int getDistanceCm() const;

    // Median sensor output voltage of the most recent reading
    float getVoltage() const;
    int getRawADC() const;

    // True if the last reading is within [minDistanceCm, maxDistanceCm]
    bool isValid() const;

    Status getStatus() const;
    static const char* statusToString(Status status);

private:
    uint8_t pin;
    int minDistanceCm;
    int maxDistanceCm;

    int distanceCm = 0;  // filtered distance, used for Sharp safety interlock
    float filteredCm = 0.0f;
    bool initialized = false;
    float outlierCandidateCm = 0.0f;
    uint8_t outlierCount = 0;
    uint8_t invalidCount = 0;
    // One-pole low-pass after the existing median-of-25 ADC filter.
    static constexpr float EMA_ALPHA = 0.35f;
    static constexpr float SPIKE_THRESHOLD_CM = 12.0f;
    static constexpr float SPIKE_CLUSTER_CM = 8.0f;
    static constexpr uint8_t CONFIRM_READINGS = 3;
    float voltage = 0.0f;
    int rawADC = 0;

    // Sampling
    static constexpr int NUM_SAMPLES = 25;
    static constexpr unsigned long SAMPLE_TIME = 50;
    unsigned long lastSample = 0;

    int readMedianADC() const;
};

#endif
