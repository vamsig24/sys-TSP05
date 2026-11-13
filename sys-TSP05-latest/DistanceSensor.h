#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>

// VL53L1X time-of-flight distance sensor (I2C, SparkFun library).
// Non-blocking: call update() from loop(); it never uses delay().
class DistanceSensor {
public:
    // timingBudgetMs: measurement time per reading (20, 33, 50, 100, 200, 500)
    // longRange: true = long distance mode (up to ~4 m), false = short (~1.3 m)
    DistanceSensor(uint16_t timingBudgetMs = 100, bool longRange = true);

    // Initializes I2C and the sensor and starts ranging.
    // Returns false if the sensor is not detected.
    bool begin();

    // Call repeatedly from loop(); reads a new measurement when one is ready.
    void update();

    // Most recent distance reading
    int getDistanceMm() const;
    float getDistanceCm() const;

    // True once per new reading (cleared when called)
    bool hasNewReading();

    // True if the sensor initialized and has not timed out
    bool isOk() const;

private:
    SFEVL53L1X sensor;

    uint16_t timingBudgetMs;
    bool longRange;

    bool initialized = false;
    bool timedOut = false;
    bool newReading = false;

    int distanceMm = 0;

    // Timing
    static constexpr unsigned long POLL_INTERVAL_MS = 5;
    static constexpr unsigned long TIMEOUT_MS = 1000;
    unsigned long lastPoll = 0;
    unsigned long lastReading = 0;
};

#endif
