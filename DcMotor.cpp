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

    // Direction corrected based on previous test; encoderInverted flips it
    bool forward = digitalRead(instance->encoderBPin) == HIGH;
    if (instance->encoderInverted) {
        forward = !forward;
    }
    if (forward) {
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

    // Driver output; motorInverted swaps direction like swapping the wires
    int out = motorInverted ? -currentPWM : currentPWM;

    if (out > 0) {

        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);

    } else if (out < 0) {

        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);

    } else {

        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
    }

    analogWrite(enaPin, abs(out));
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

    // Open-loop test: fixed PWM, no controller
    if (openLoop) {
        integral = 0.0f;
        setMotor(openLoopPWM);
        return;
    }

    // Stop command
    if (targetRPM == 0.0f) {

        integral = 0.0f;
        setMotor(0);
        return;
    }

    // PI controller
    float error = targetRPM - measuredRPM;

    integral += error * dt;
    integral = constrain(integral, -30.0f, 30.0f);

    float ff = FF_OFFSET_PWM + fabs(targetRPM) * FF_PWM_PER_RPM;
    if (fabs(measuredRPM) < STALL_RPM) {
        ff = max(ff, KICK_PWM);   // break away from standstill
        integral = 0.0f;
    }
    if (targetRPM < 0.0f) ff = -ff;

    float output = ff + Kp * error + Ki * integral;

    setMotor((int)output);
}

// ====================================
// Polarity and open-loop test
// ====================================

void DCMotor::setInverted(bool motorInv, bool encoderInv) {
    noInterrupts();
    encoderInverted = encoderInv;
    interrupts();
    motorInverted = motorInv;
    integral = 0.0f;
}

void DCMotor::setOpenLoopPWM(int pwm) {
    openLoop = true;
    openLoopPWM = constrain(pwm, -MAX_PWM, MAX_PWM);
}

void DCMotor::setClosedLoop() {
    openLoop = false;
    openLoopPWM = 0;
    integral = 0.0f;
}

// ====================================
// Getters
// ====================================

float DCMotor::getRPM() const {
    return measuredRPM;
}