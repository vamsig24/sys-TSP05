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

    float volts = readMedianADC() * 5.0f / 1023.0f;
    voltage = volts;

    if (volts <= 0.0f) {
        distanceCm = 0;
        return;
    }

    // GP2Y0A21YK0F (model 1080) fit, same as the SharpIR library
    distanceCm = (int)(27.728f * powf(volts, -1.2045f));
}

// ====================================
// Getters
// ====================================

int SharpIRSensor::getDistanceCm() const {
    return distanceCm;
}

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
