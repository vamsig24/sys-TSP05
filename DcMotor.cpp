#include "DcMotor.h"

// Static instance pointer for interrupt handling
DCMotor* DCMotor::instance = nullptr;

// ====================================
// Constructor
// ====================================

DCMotor::DCMotor(
    uint8_t enaPin,
    uint8_t in1Pin,
    uint8_t in2Pin,
    uint8_t encoderAPin,
    uint8_t encoderBPin
)
    : enaPin(enaPin),
      in1Pin(in1Pin),
      in2Pin(in2Pin),
      encoderAPin(encoderAPin),
      encoderBPin(encoderBPin)
{
}

// ====================================
// Initialization
// ====================================

void DCMotor::begin() {

    pinMode(enaPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);

    pinMode(encoderAPin, INPUT_PULLUP);
    pinMode(encoderBPin, INPUT_PULLUP);

    // Motor initially stopped
    analogWrite(enaPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);

    // Register instance for interrupt callback
    instance = this;

    attachInterrupt(
        digitalPinToInterrupt(encoderAPin),
        encoderISR,
        RISING
    );

    lastUpdate = millis();
}

// ====================================
// Encoder interrupt
// ====================================

void DCMotor::encoderISR() {

    if (instance == nullptr) {
        return;
    }

    // Direction corrected based on previous test
    if (digitalRead(instance->encoderBPin) == HIGH) {
        instance->encoderCount++;
    } else {
        instance->encoderCount--;
    }
}

// ====================================
// Speed command
// ====================================

void DCMotor::setTargetRPM(float rpm) {

    rpm = constrain(rpm, -100.0f, 100.0f);

    // Reset integrator when target changes
    if (rpm != targetRPM) {
        integral = 0.0f;
    }

    targetRPM = rpm;
}

// ====================================
// PI gains
// ====================================

void DCMotor::setPIDGains(float kp, float ki) {
    Kp = kp;
    Ki = ki;

    integral = 0.0f;
}

// ====================================
// Motor output
// ====================================

void DCMotor::setMotor(int pwm) {

    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    // Limit PWM change per update
    int change = constrain(
        pwm - currentPWM,
        -MAX_PWM_CHANGE,
        MAX_PWM_CHANGE
    );

    currentPWM += change;

    if (currentPWM > 0) {

        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);

    } else if (currentPWM < 0) {

        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);

    } else {

        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
    }

    analogWrite(enaPin, abs(currentPWM));
}

// ====================================
// Closed-loop update
// ====================================

void DCMotor::update() {

    unsigned long now = millis();

    if (now - lastUpdate < SAMPLE_TIME) {
        return;
    }

    float dt = (now - lastUpdate) / 1000.0f;
    lastUpdate = now;

    // Read encoder count atomically
    noInterrupts();
    long pulses = encoderCount;
    encoderCount = 0;
    interrupts();

    // Calculate signed RPM
    measuredRPM =
        (pulses / PULSES_PER_REV) *
        (60.0f / dt);

    // Stop command
    if (targetRPM == 0.0f) {

        integral = 0.0f;
        setMotor(0);
        return;
    }

    // PI controller
    float error = targetRPM - measuredRPM;

    integral += error * dt;
    integral = constrain(integral, -150.0f, 150.0f);

    float output = Kp * error + Ki * integral;

    setMotor((int)output);
}

// ====================================
// Getters
// ====================================

float DCMotor::getRPM() const {
    return measuredRPM;
}