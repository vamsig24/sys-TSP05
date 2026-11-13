#include "DistanceSensor.h"

// ====================================
// Constructor
// ====================================

DistanceSensor::DistanceSensor(uint16_t timingBudgetMs, bool longRange)
    : timingBudgetMs(timingBudgetMs),
      longRange(longRange)
{
}

// ====================================
// Initialization
// ====================================

bool DistanceSensor::begin() {

    Wire.begin();

    // SparkFun library returns 0 on success
    if (sensor.begin() != 0) {
        initialized = false;
        return false;
    }

    if (longRange) {
        sensor.setDistanceModeLong();
    } else {
        sensor.setDistanceModeShort();
    }

    sensor.setTimingBudgetInMs(timingBudgetMs);
    sensor.startRanging();

    initialized = true;
    timedOut = false;
    lastPoll = millis();
    lastReading = lastPoll;

    return true;
}

// ====================================
// Non-blocking update
// ====================================

void DistanceSensor::update() {

    if (!initialized) {
        return;
    }

    unsigned long now = millis();

    // Limit I2C traffic so other update() calls are not delayed
    if (now - lastPoll < POLL_INTERVAL_MS) {
        return;
    }
    lastPoll = now;

    if (!sensor.checkForDataReady()) {
        if (now - lastReading > TIMEOUT_MS) {
            timedOut = true;
        }
        return;
    }

    distanceMm = sensor.getDistance();

    // Clear interrupt for next reading
    sensor.clearInterrupt();

    lastReading = now;
    timedOut = false;
    newReading = true;
}

// ====================================
// Getters
// ====================================

int DistanceSensor::getDistanceMm() const {
    return distanceMm;
}

float DistanceSensor::getDistanceCm() const {
    return distanceMm / 10.0f;
}

bool DistanceSensor::hasNewReading() {
    bool result = newReading;
    newReading = false;
    return result;
}

bool DistanceSensor::isOk() const {
    return initialized && !timedOut;
}
