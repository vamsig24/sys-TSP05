#ifndef TSP05_SENSOR_MAPPING_H
#define TSP05_SENSOR_MAPPING_H
#include <stdint.h>

// Small, allocation-free mapping helpers: safe on an Arduino Uno.
// The raw sensor samples are filtered by the existing sensor drivers.
namespace TSP05Mapping {
inline float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
inline int flexToServo(float percent) {
    return (int)(clamp(percent, 0.0f, 100.0f) * 1.8f + 0.5f);
}
inline float potToStepper(float adc, float minADC, float maxADC) {
    if (maxADC - minADC < 1.0f) return 0.0f;
    return 360.0f * clamp((adc - minADC) / (maxADC - minADC), 0.0f, 1.0f);
}

// Hysteresis plus command rate limiting prevents servo chattering near
// unchanged flex angles. Returning -1 means 'do not send another command'.
// The full 0 and 180 degree positions remain achievable.
class ServoDeadband {
public:
    void reset(unsigned long now) {
        lastSampleMs_ = now - SAMPLE_INTERVAL_MS;
        lastWriteMs_ = now - WRITE_INTERVAL_MS;
        pendingDirection_ = 0;
        consecutive_ = 0;
    }
    int command(float bendPercent, int actualCommandDeg, unsigned long now) {
        if ((unsigned long)(now - lastSampleMs_) < SAMPLE_INTERVAL_MS) return -1;
        lastSampleMs_ = now;
        int target = flexToServo(bendPercent);
        // Schmitt-style hysteresis at both mechanical ends.
        if (actualCommandDeg == 0 && target <= ZERO_EXIT_DEG) target = 0;
        else if (target <= ZERO_ENTER_DEG) target = 0;
        if (actualCommandDeg == 180 && target >= FULL_EXIT_DEG) target = 180;
        else if (target >= FULL_ENTER_DEG) target = 180;
        int delta = target - actualCommandDeg;
        if (delta < 0) delta = -delta;
        if (delta < DEAD_BAND_DEG) {
            pendingDirection_ = 0;
            consecutive_ = 0;
            return -1;
        }
        int direction = target > actualCommandDeg ? 1 : -1;
        if (direction == pendingDirection_) ++consecutive_;
        else {
            pendingDirection_ = direction;
            consecutive_ = 1;
        }
        // Require two successive samples outside the deadband in the same
        // direction; a single noisy bend spike cannot move the servo.
        if (consecutive_ < 2 ||
            (unsigned long)(now - lastWriteMs_) < WRITE_INTERVAL_MS) return -1;
        pendingDirection_ = 0;
        consecutive_ = 0;
        lastWriteMs_ = now;
        return target;
    }
    static constexpr int DEAD_BAND_DEG = 8;
    static constexpr int ZERO_ENTER_DEG = 7;
    static constexpr int ZERO_EXIT_DEG = 16;
    static constexpr int FULL_ENTER_DEG = 177;
    static constexpr int FULL_EXIT_DEG = 169;
    static constexpr unsigned long SAMPLE_INTERVAL_MS = 60;
    static constexpr unsigned long WRITE_INTERVAL_MS = 180;
private:
    unsigned long lastSampleMs_ = 0;
    unsigned long lastWriteMs_ = 0;
    int8_t pendingDirection_ = 0;
    uint8_t consecutive_ = 0;
};
}
#endif
