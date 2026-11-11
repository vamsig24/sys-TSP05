#include "SharpIRSensor.h"
#include <math.h>

// ====================================
// Constructor
// ====================================

SharpIRSensor::SharpIRSensor(uint8_t pin,
                             int minDistanceCm,
                             int maxDistanceCm)
    : pin(pin),
      minDistanceCm(minDistanceCm),
      maxDistanceCm(maxDistanceCm)
{
}

// ====================================
// Initialization
// ====================================

void SharpIRSensor::begin() {

    pinMode(pin, INPUT);
    initialized = false;
    outlierCount = 0;
    invalidCount = 0;
    lastSample = millis() - SAMPLE_TIME;  // take first reading immediately
}

// ====================================
// Sampling
// ====================================

int SharpIRSensor::readMedianADC() const {

    int samples[NUM_SAMPLES];

    for (int i = 0; i < NUM_SAMPLES; i++) {
        samples[i] = analogRead(pin);
    }

    // Insertion sort
    for (int i = 1; i < NUM_SAMPLES; i++) {
        for (int j = i; j > 0 && samples[j - 1] > samples[j]; j--) {
            int t = samples[j];
            samples[j] = samples[j - 1];
            samples[j - 1] = t;
        }
    }

    return samples[NUM_SAMPLES / 2];
}

void SharpIRSensor::update() {

    unsigned long now = millis();

    if (now - lastSample < SAMPLE_TIME) {
        return;
    }
    lastSample = now;

    rawADC = readMedianADC();
    float volts = rawADC * 5.0f / 1023.0f;
    voltage = volts;

    // Preserve the raw electrical reading. Filter only the converted distance.
    // A zero/disconnected ADC reading must never enter the power-law fit.
    if (rawADC <= 1) {
        if (invalidCount < 255) invalidCount++;
        if (invalidCount >= 5) {
            distanceCm = 0; // explicitly INVALID_READING, no bogus huge range
            initialized = false;
        }
        return;
    }
    invalidCount = 0;

    // GP2Y0A21YK0F (1080), unchanged conversion formula.
    const float candidate = 27.728f * powf(volts, -1.2045f);
    if (!isfinite(candidate)) return;

    if (!initialized) {
        filteredCm = candidate;
        initialized = true;
    } else if (fabsf(candidate - filteredCm) > SPIKE_THRESHOLD_CM) {
        // Reject isolated large up/down excursions. If the target ACTUALLY
        // moved, three similar readings in a row authorize the new value.
        if (outlierCount == 0 ||
            fabsf(candidate - outlierCandidateCm) > SPIKE_CLUSTER_CM) {
            outlierCandidateCm = candidate;
            outlierCount = 1;
        } else if (outlierCount < CONFIRM_READINGS) {
            outlierCount++;
        }
        if (outlierCount >= CONFIRM_READINGS) {
            filteredCm = candidate; // confirmed real step: respond promptly
            outlierCount = 0;
        }
    } else {
        outlierCount = 0;
        filteredCm += EMA_ALPHA * (candidate - filteredCm);
    }

    distanceCm = (int)(filteredCm + 0.5f);
}

// ====================================
// Getters
// ====================================

int SharpIRSensor::getDistanceCm() const {
    return distanceCm;
}

int SharpIRSensor::getRawADC() const { return rawADC; }

float SharpIRSensor::getVoltage() const {
    return voltage;
}

bool SharpIRSensor::isValid() const {
    return getStatus() == VALID;
}

SharpIRSensor::Status SharpIRSensor::getStatus() const {

    if (distanceCm <= 0) {
        return INVALID_READING;
    }
    if (distanceCm < minDistanceCm) {
        return TOO_CLOSE;
    }
    if (distanceCm > maxDistanceCm) {
        return TOO_FAR;
    }
    return VALID;
}

const char* SharpIRSensor::statusToString(Status status) {

    switch (status) {
        case TOO_CLOSE: return "TOO_CLOSE";
        case TOO_FAR:   return "TOO_FAR";
        case VALID:     return "VALID";
        default:        return "INVALID_READING";
    }
}
