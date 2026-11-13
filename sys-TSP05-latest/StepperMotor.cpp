
    
  
#include "StepperMotor.h"
#include <math.h>

StepperMotor::StepperMotor(uint8_t dirPin, uint8_t stepPin,
                           long microstepsPerRevolution)
    : dirPin_(dirPin),
      stepPin_(stepPin),
      microstepsPerRevolution_(microstepsPerRevolution > 0
                                  ? microstepsPerRevolution : 1600) {}

void StepperMotor::begin() {
    pinMode(dirPin_, OUTPUT);
    pinMode(stepPin_, OUTPUT);
    digitalWrite(dirPin_, LOW);
    digitalWrite(stepPin_, LOW);
    positionSteps_ = 0;
    targetSteps_ = 0;
    pendingDirection_ = 0;
    pulseHigh_ = false;
    lastEdgeUs_ = micros();
}

long StepperMotor::degreesToSteps(float degrees) const {
    return lroundf(degrees * (float)microstepsPerRevolution_ / 360.0f);
}

void StepperMotor::setTargetDegrees(float degrees) {
    targetSteps_ = degreesToSteps(degrees);
}

void StepperMotor::moveDegrees(float degrees) {
    targetSteps_ += degreesToSteps(degrees);
}

void StepperMotor::setSpeed(float microstepsPerSecond) {
    if (microstepsPerSecond < 1.0f) microstepsPerSecond = 1.0f;
    if (microstepsPerSecond > 2000.0f) microstepsPerSecond = 2000.0f;

    unsigned long periodUs = (unsigned long)(1000000.0f / microstepsPerSecond);
    lowTimeUs_ = periodUs - HIGH_TIME_US;
}

void StepperMotor::update() {
    unsigned long now = micros();

    // Finish any active pulse first, even if the target changes mid-pulse.
    if (pulseHigh_) {
        if ((unsigned long)(now - lastEdgeUs_) < HIGH_TIME_US) return;
        digitalWrite(stepPin_, LOW);
        pulseHigh_ = false;
        positionSteps_ += pendingDirection_;
        lastEdgeUs_ = now;
        return;
    }

    if (positionSteps_ == targetSteps_) return;
    if ((unsigned long)(now - lastEdgeUs_) < lowTimeUs_) return;

    int8_t direction = (targetSteps_ > positionSteps_) ? 1 : -1;
    // Give the driver time after switching DIR before the STEP rising edge.
    // The delay is non-blocking: it is handled by the next update() call.
    if (pendingDirection_ != direction) {
        digitalWrite(dirPin_, direction > 0 ? HIGH : LOW);
        pendingDirection_ = direction;
        lastEdgeUs_ = now;
        return;
    }

    // Honor a minimum direction setup time, including after a reversal.
    // lowTimeUs_ >= 490 us with the supported speed range.
    digitalWrite(stepPin_, HIGH);
    pulseHigh_ = true;
    lastEdgeUs_ = now;
}

float StepperMotor::getPositionDegrees() const {
    return (float)positionSteps_ * 360.0f / (float)microstepsPerRevolution_;
}

bool StepperMotor::isMoving() const {
    return pulseHigh_ || positionSteps_ != targetSteps_;
}


float StepperMotor::getTargetDegrees() const {
    return (float)targetSteps_ * 360.0f / (float)microstepsPerRevolution_;
}
