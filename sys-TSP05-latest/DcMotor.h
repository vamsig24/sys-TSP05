#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>

class DCMotor {
public:
    DCMotor(
        uint8_t enaPin,
        uint8_t in1Pin,
        uint8_t in2Pin,
        uint8_t encoderAPin,
        uint8_t encoderBPin
    );

    void begin();

    // Command speed in RPM (-100 to 100)
    void setTargetRPM(float rpm);

    // Call repeatedly from loop()
    void update();

    // Current measured speed in RPM
    float getRPM() const;

    // Incremental output-shaft position since power-up (encoder feedback).
    // Set PULSES_PER_REV to the MEASURED rising edges on encoder A per OUTPUT revolution.
    float getPositionDegrees() const;
    void moveRelativeDegrees(float deltaDegrees); // relative to current measured position
    void cancelPositionMove();
    void emergencyStop(); // immediate PWM=0, independent of slew limiter
    bool isPositionMode() const { return positionMode; }
    float getTargetPositionDegrees() const { return targetPositionDeg; }
    float getActiveTargetRPM() const { return activeTargetRPM; }
    bool positionReached() const;


    // Change PI gains
    void setPIDGains(float kp, float ki, float kd = 0.0f);

    // Polarity: motorInverted swaps the driver direction (same as swapping
    // the motor wires), encoderInverted flips the encoder count sign.
    // For the PI loop, positive PWM must give positive measured RPM.
    void setInverted(bool motorInverted, bool encoderInverted);
    bool isMotorInverted() const { return motorInverted; }
    bool isEncoderInverted() const { return encoderInverted; }

    // Open-loop test: drive a fixed PWM (-230..230), bypassing the PI loop,
    // until setClosedLoop() is called. Speed is still measured.
    void setOpenLoopPWM(int pwm);
    void setClosedLoop();
    bool isOpenLoop() const { return openLoop; }

    // PWM currently applied (logical sign, before motor inversion)
    int getPWM() const { return currentPWM; }

    // Stall protection (closed loop): if the motor is driven near full power but the
    // encoder sees no motion for STALL_TIMEOUT_MS, the output is cut.
    // Cleared by clearStall(), a zero target or a direction change.
    bool isStalled() const { return stalled; }
    void clearStall();

private:
    // Motor driver pins
    uint8_t enaPin;
    uint8_t in1Pin;
    uint8_t in2Pin;

    // Encoder pins
    uint8_t encoderAPin;
    uint8_t encoderBPin;

    // Encoder
    volatile long encoderCount = 0;
    volatile long totalEncoderCount = 0;
    volatile bool encoderInverted = false;
    bool motorInverted = false;

    // Open-loop test mode
    bool openLoop = false;
    int openLoopPWM = 0;

    static DCMotor* instance;
    static void encoderISR();

    // Motor parameters
    // IMPORTANT: calibrate this on the actual geared motor (A rising edges, x1).
    // This is a starting assumption only, NOT a verified kit specification.
    static constexpr float PULSES_PER_REV = 210.0f;

    // PI controller
    float targetRPM = 0.0f;
    float measuredRPM = 0.0f;
    float activeTargetRPM = 0.0f;
    float targetPositionDeg = 0.0f;
    bool positionMode = false;
    bool positionComplete = false; // Latches after reaching/crossing goal; never hunt/reverse
    int8_t positionDirection = 0;  // Immutable direction for this relative move
    float previousRPM = 0.0f;
    static constexpr float POSITION_KP = 0.45f;  // rpm per degree of remaining error
    static constexpr float POSITION_KD = 0.025f; // rpm per degree/sec
    static constexpr float POSITION_TOLERANCE_DEG = 3.0f;
    static constexpr float POSITION_MAX_RPM = 35.0f;
    // No forced high minimum approaching the encoder goal (old value: 20 RPM).
    static constexpr float POSITION_MIN_MOVING_RPM = 8.0f;

    float Kp = 1.5f;
    float Ki = 5.0f;
    float Kd = 0.0f;

    // Feedforward: pwm = sign * (FF_OFFSET_PWM + |rpm| * FF_PWM_PER_RPM),
    // fitted to the TSP05 motor on its fixed supply (2026-11-09):
    // 30 rpm ~ PWM 65, 60 rpm ~ PWM 95, 90 rpm ~ PWM 135, saturates
    // ~115 rpm above PWM 180. The PI loop corrects the rest.
    static constexpr float FF_OFFSET_PWM = 30.0f;
    static constexpr float FF_PWM_PER_RPM = 1.1f;

    // Breakaway: while the motor is not turning, the output ramps up from
    // the feedforward by KICK_STEP_PWM per sample until it moves (the force
    // needed differs per direction and supply). If it is still not turning
    // at STALL_PWM or more for STALL_TIMEOUT_MS, it is stalled.
    static constexpr float KICK_STEP_PWM = 15.0f;
    static constexpr int STALL_PWM = 200;
    float kickPWM = 0.0f;
    static constexpr float STALL_RPM = 3.0f;
    static constexpr unsigned long STALL_TIMEOUT_MS = 1500;

    bool stalled = false;
    unsigned long stallStart = 0;   // 0 = not stalling

    float integral = 0.0f;

    // PWM
    static constexpr int MAX_PWM = 230;
    static constexpr int MAX_PWM_CHANGE = 30;

    int currentPWM = 0;

    // Timing
    static constexpr unsigned long SAMPLE_TIME = 200;
    unsigned long lastUpdate = 0;

    void setMotor(int pwm);
};

#endif