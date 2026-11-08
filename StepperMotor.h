


    
  
#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include <Arduino.h>

// STEP/DIR stepper driver interface (e.g. EasyDriver).
// Position is estimated from commanded steps; no encoder or homing.
class StepperMotor {
public:
    StepperMotor(uint8_t dirPin, uint8_t stepPin,
                 long microstepsPerRevolution = 1600);

    void begin();

    // Absolute angle relative to the position at startup (0 degrees).
    void setTargetDegrees(float degrees);

    // Relative to the previously commanded target (queues the movement).
    void moveDegrees(float degrees);

    // Approximate microstep rate. Defaults to 1 / (1510 us) = 662 steps/s.
    // Clamped to a range appropriate for this polling implementation.
    void setSpeed(float microstepsPerSecond);

    // Call as frequently as possible from loop(); never uses delay().
    void update();

    float getPositionDegrees() const;
    bool isMoving() const;

private:
    uint8_t dirPin_;
    uint8_t stepPin_;
    long microstepsPerRevolution_;

    long positionSteps_ = 0;
    long targetSteps_ = 0;
    int8_t pendingDirection_ = 0;
    bool pulseHigh_ = false;

    unsigned long lastEdgeUs_ = 0;
    unsigned long lowTimeUs_ = 1500;
    static const unsigned long HIGH_TIME_US = 10;
    static const unsigned long DIR_SETUP_US = 10;

    long degreesToSteps(float degrees) const;
};

#endif