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
        instance->totalEncoderCount++;
    } else {
        instance->encoderCount--;
        instance->totalEncoderCount--;
    }
}

// ====================================
// Speed command
// ====================================

void DCMotor::setTargetRPM(float rpm) {

    positionMode = false;
    positionComplete = false;
    positionDirection = 0;
    rpm = constrain(rpm, -100.0f, 100.0f);

    // Reset integrator when target changes
    if (rpm != targetRPM) {
        integral = 0.0f;
    }

    // Stopping or reversing clears a stall
    if (rpm == 0.0f || (rpm > 0.0f) != (targetRPM > 0.0f)) {
        clearStall();
    }

    targetRPM = rpm;
}

// ====================================
// PI gains
// ====================================

void DCMotor::setPIDGains(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;

    integral = 0.0f;
}

// ====================================
// Motor output
// ====================================

void DCMotor::setMotor(int pwm) {

    pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

    // Stopping must be immediate, especially for the Sharp safety interlock;
    // only acceleration and nonzero direction changes are rate-limited.
    if (pwm == 0) {
        currentPWM = 0;
    } else {
        int change = constrain(pwm - currentPWM,
                               -MAX_PWM_CHANGE, MAX_PWM_CHANGE);
        currentPWM += change;
    }

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

    // Check encoder position EVERY iteration, not just at the 200 ms RPM
    // sample. At 20 RPM the motor can move 24 degrees in 200 ms, so an
    // RPM-only position check can skip right over a +/-3 degree target.
    // Once the goal window is reached or crossed, stop and latch. Do NOT
    // command an automatic reverse on overshoot: it caused oscillation.
    if (positionMode && !positionComplete) {
        const float remaining = targetPositionDeg - getPositionDegrees();
        if (positionDirection == 0 ||
            remaining * positionDirection <= POSITION_TOLERANCE_DEG) {
            positionComplete = true;
            activeTargetRPM = 0.0f;
            integral = 0.0f;
            kickPWM = 0.0f;
            setMotor(0);
        }
    }

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

    // Compute a new relative position goal from the counted encoder pulses.
    // In position mode the outer PD loop requests signed velocity; the
    // inner encoder PID loop below closes the speed loop.
    if (positionMode) {
        if (positionComplete) {
            activeTargetRPM = 0.0f;
        } else {
            float remaining = targetPositionDeg - getPositionDegrees();
            // Approach in the ORIGINAL direction only. Any encoder overshoot
            // is handled by the latched stop above, never reverse hunting.
            float request = POSITION_KP * fabs(remaining);
            request = constrain(request, 0.0f, POSITION_MAX_RPM);
            if (request < POSITION_MIN_MOVING_RPM) {
                request = POSITION_MIN_MOVING_RPM;
            }
            activeTargetRPM = positionDirection * request;
        }
    } else {
        activeTargetRPM = targetRPM;
    }

    // Open-loop test: fixed PWM, no controller
    if (openLoop) {
        integral = 0.0f;
        setMotor(openLoopPWM);
        return;
    }

    // Stall protection: driven hard, no motion
    if (!stalled && abs(currentPWM) >= STALL_PWM &&
        fabs(measuredRPM) < STALL_RPM) {
        if (stallStart == 0) {
            stallStart = now;
        } else if (now - stallStart >= STALL_TIMEOUT_MS) {
            stalled = true;
        }
    } else if (fabs(measuredRPM) >= STALL_RPM) {
        stallStart = 0;
    }

    // Stop command (or stalled)
    if (activeTargetRPM == 0.0f || stalled) {

        integral = 0.0f;
        kickPWM = 0.0f;
        setMotor(0);
        return;
    }

    // PI controller
    float error = activeTargetRPM - measuredRPM;

    integral += error * dt;
    integral = constrain(integral, -30.0f, 30.0f);

    float ff = FF_OFFSET_PWM + fabs(activeTargetRPM) * FF_PWM_PER_RPM;
    if (fabs(measuredRPM) < STALL_RPM) {
        // Not turning: ramp up until it breaks away
        kickPWM = (kickPWM == 0.0f) ? ff : kickPWM + KICK_STEP_PWM;
        kickPWM = min(kickPWM, (float)MAX_PWM);
        ff = max(ff, kickPWM);
        integral = 0.0f;
    } else {
        kickPWM = 0.0f;
    }
    if (activeTargetRPM < 0.0f) ff = -ff;

    // Derivative on the encoder measurement avoids derivative kick when
    // moving between position/speed setpoints.
    float derivative = (measuredRPM - previousRPM) / dt;
    previousRPM = measuredRPM;
    float output = ff + Kp * error + Ki * integral - Kd * derivative;

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
    positionMode = false;
    positionComplete = false;
    positionDirection = 0;
    openLoop = true;
    openLoopPWM = constrain(pwm, -MAX_PWM, MAX_PWM);
}

void DCMotor::clearStall() {
    stalled = false;
    stallStart = 0;
}

void DCMotor::setClosedLoop() {
    openLoop = false;
    openLoopPWM = 0;
    previousRPM = measuredRPM;
    integral = 0.0f;
}

// ====================================
// Getters
// ====================================

float DCMotor::getPositionDegrees() const {
    noInterrupts();
    long pulses = totalEncoderCount;
    interrupts();
    return pulses * (360.0f / PULSES_PER_REV);
}

void DCMotor::moveRelativeDegrees(float deltaDegrees) {
    // Always relative to actual measured position, not a previous target.
    targetPositionDeg = getPositionDegrees() + deltaDegrees;
    positionDirection = deltaDegrees > 0.0f ? 1 : (deltaDegrees < 0.0f ? -1 : 0);
    positionMode = true;
    positionComplete = (positionDirection == 0 ||
                        fabs(deltaDegrees) <= POSITION_TOLERANCE_DEG);
    activeTargetRPM = 0.0f;
    openLoop = false;
    integral = 0.0f;
    kickPWM = 0.0f;
    setMotor(0); // stop old speed command before new relative move
    clearStall();
}

void DCMotor::emergencyStop() {
    cancelPositionMove();
    openLoop = false;
    openLoopPWM = 0;
    currentPWM = 0;
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(enaPin, 0);
}

void DCMotor::cancelPositionMove() {
    positionMode = false;
    positionComplete = false;
    positionDirection = 0;
    targetRPM = 0.0f;
    activeTargetRPM = 0.0f;
    integral = 0.0f;
}

bool DCMotor::positionReached() const {
    // A completed move is latched; motor remains off, even if it coasts past
    // the target. For accuracy, compare dc_pos to dc_pos_tgt separately.
    return positionMode && positionComplete && fabs(measuredRPM) < STALL_RPM;
}

float DCMotor::getRPM() const {
    return measuredRPM;
}