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

    static DCMotor* instance;
    static void encoderISR();

    // Motor parameters
    static constexpr float PULSES_PER_REV = 210.0f;

    // PI controller
    float targetRPM = 0.0f;
    float measuredRPM = 0.0f;

    float Kp = 1.0f;
    float Ki = 1.5f;

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