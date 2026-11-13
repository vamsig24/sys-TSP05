#include "Potentiometer.h"

// ====================================
// Constructor
// ====================================

Potentiometer::Potentiometer(uint8_t pin, float adcReference)
    : pin(pin),
      adcReference(adcReference)
{
}

// ====================================
// Initialization
// ====================================

void Potentiometer::begin() {

    total = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        samples[i] = analogRead(pin);
        total += samples[i];
    }

    rawADC = samples[SAMPLE_COUNT - 1];
    sampleIndex = 0;
    lastSample = millis();
}

// ====================================
// Moving average sampling
// ====================================

void Potentiometer::update() {

    unsigned long now = millis();

    if (now - lastSample < SAMPLE_TIME) {
        return;
    }
    lastSample = now;

    rawADC = analogRead(pin);

    // Replace oldest sample in the running total
    total -= samples[sampleIndex];
    samples[sampleIndex] = rawADC;
    total += rawADC;

    sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
}

// ====================================
// Getters
// ====================================

int Potentiometer::getRawADC() const {
    return rawADC;
}

float Potentiometer::getFilteredADC() const {
    return (float)total / SAMPLE_COUNT;
}

float Potentiometer::getVoltage() const {
    return getFilteredADC() * (adcReference / 1023.0f);
}

int Potentiometer::getPositionPercent() const {
    return map((int)(getFilteredADC() + 0.5f), 0, 1023, 0, 100);
}
