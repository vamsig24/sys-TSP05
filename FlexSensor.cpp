#include "FlexSensor.h"

// ====================================
// Constructor
// ====================================

FlexSensor::FlexSensor(uint8_t pin,
                       float fixedResistance,
                       float flatResistance,
                       float bentResistance)
    : pin(pin),
      fixedResistance(fixedResistance),
      flatResistance(flatResistance),
      bentResistance(bentResistance)
{
}

// ====================================
// Initialization
// ====================================

void FlexSensor::begin() {

    for (int i = 0; i < MEDIAN_SIZE; i++) {
        samples[i] = analogRead(pin);
    }

    smoothed = analogRead(pin);
    rawADC = (int)smoothed;
    lastSample = millis();
}

// ====================================
// Filtered sampling
// ====================================

int FlexSensor::median() const {

    int sorted[MEDIAN_SIZE];
    memcpy(sorted, samples, sizeof(samples));

    // Insertion sort
    for (int i = 1; i < MEDIAN_SIZE; i++) {
        for (int j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
            int t = sorted[j];
            sorted[j] = sorted[j - 1];
            sorted[j - 1] = t;
        }
    }

    return sorted[MEDIAN_SIZE / 2];
}

void FlexSensor::update() {

    unsigned long now = millis();

    if (now - lastSample < SAMPLE_TIME) {
        return;
    }
    lastSample = now;

    rawADC = analogRead(pin);
    samples[sampleIdx] = rawADC;
    sampleIdx = (sampleIdx + 1) % MEDIAN_SIZE;

    smoothed += EMA_ALPHA * (median() - smoothed);
}

// ====================================
// Conversions
// ====================================

float FlexSensor::getResistanceOhms() const {

    // Avoid division by zero / negative resistance at the rails
    float adc = constrain(smoothed, 1.0f, 1022.0f);

    // 5V -- Flex Sensor -- pin -- fixed resistor -- GND
    return fixedResistance * (1023.0f / adc - 1.0f);
}

float FlexSensor::getADC() const {
    return smoothed;
}

int FlexSensor::getRawADC() const {
    return rawADC;
}

float FlexSensor::readResistance() const {
    return getResistanceOhms() / 1000.0f;
}

float FlexSensor::getForcePercent() const {

    float force =
        (getResistanceOhms() - flatResistance) * 100.0f /
        (bentResistance - flatResistance);

    return constrain(force, 0.0f, 100.0f);
}

int FlexSensor::getBendAngle() const {
    return (int)(getForcePercent() * 1.8f + 0.5f);
}

void FlexSensor::setCalibration(float flatOhms, float bentOhms) {
    if (bentOhms > flatOhms) {
        flatResistance = flatOhms;
        bentResistance = bentOhms;
    }
}
