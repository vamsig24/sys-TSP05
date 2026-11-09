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

    // Change PI gains
    void setPIDGains(float kp, float ki);

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
    volatile bool encoderInverted = false;
    bool motorInverted = false;

    // Open-loop test mode
    bool openLoop = false;
    int openLoopPWM = 0;

    static DCMotor* instance;
    static void encoderISR();

    // Motor parameters
    static constexpr float PULSES_PER_REV = 210.0f;

    // PI controller
    float targetRPM = 0.0f;
    float measuredRPM = 0.0f;

    float Kp = 1.5f;
    float Ki = 3.0f;

    // Feedforward: pwm = sign * (FF_OFFSET_PWM + |rpm| * FF_PWM_PER_RPM),
    // fitted to the TSP05 motor once it is turning (PWM 80 ~ 21 rpm,
    // 100 ~ 32 rpm, 120 ~ 38 rpm, flat ~50 rpm above 160). From standstill
    // it needs ~140 to break away, so KICK_PWM is used until it moves.
    // The PI loop corrects the rest.
    static constexpr float FF_OFFSET_PWM = 55.0f;
    static constexpr float FF_PWM_PER_RPM = 1.6f;
    static constexpr float KICK_PWM = 150.0f;
    static constexpr float STALL_RPM = 3.0f;

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