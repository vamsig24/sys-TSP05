#include <cstdio>
#include <cassert>
#include <cmath>
#include "SensorMapping.h"
int main() {
    using namespace TSP05Mapping;
    assert(flexToServo(-1) == 0);
    assert(flexToServo(50) == 90);
    assert(flexToServo(100) == 180);
    assert(flexToServo(200) == 180);
    assert(std::fabs(potToStepper(120, 120, 920) - 0.0f) < 0.1f);
    assert(std::fabs(potToStepper(520, 120, 920) - 180.0f) < 0.1f);
    assert(std::fabs(potToStepper(920, 120, 920) - 360.0f) < 0.1f);
    assert(std::fabs(potToStepper(1023, 120, 920) - 360.0f) < 0.1f);

    ServoDeadband follow;
    follow.reset(1000);
    assert(follow.command(0, 90, 1000) == -1);
    assert(follow.command(0, 90, 1060) == 0);
    // 4 degree noise or 13 degree noise around 0 cannot leave zero.
    assert(follow.command(4.0f/1.8f, 0, 1120) == -1);
    assert(follow.command(13.0f/1.8f, 0, 1180) == -1);
    // A single noisy spike must NOT move the servo.
    assert(follow.command(20.0f/1.8f, 0, 1240) == -1);
    assert(follow.command(7.0f/1.8f, 0, 1300) == -1);
    assert(follow.command(20.0f/1.8f, 0, 1360) == -1);
    assert(follow.command(20.0f/1.8f, 0, 1420) == 20);
    // Variations smaller than 8 degrees are ignored even after pause.
    assert(follow.command(24.0f/1.8f, 20, 1640) == -1);
    // A deliberate bigger bend makes the servo move again after confirmation.
    assert(follow.command(35.0f/1.8f, 20, 1800) == -1);
    assert(follow.command(35.0f/1.8f, 20, 1860) == 35);
    assert(follow.command(100.0f, 35, 2100) == -1);
    assert(follow.command(100.0f, 35, 2160) == 180);
    assert(follow.command(172.0f/1.8f, 180, 2400) == -1);
    assert(follow.command(155.0f/1.8f, 180, 2600) == -1);
    assert(follow.command(155.0f/1.8f, 180, 2660) == 155);
    puts("PASS: flex->servo deadband and endpoint hysteresis; pot->stepper calibrated 0..360 mapping");
}
