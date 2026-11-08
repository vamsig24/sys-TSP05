#include <Servo.h>
#include "DcMotor.h"
#include "StepperMotor.h"

// ====================================
// Pin configuration
// ====================================

// DC motor: ENA, IN1, IN2, Encoder A, Encoder B
DCMotor dcMotor(6, 8, 7, 2, 3);

// Stepper motor: DIR, STEP
StepperMotor stepperMotor(13, 12);

// Servo motor
const int SERVO_PIN = 10;
Servo servoMotor;

// ====================================
// Test configuration
// ====================================

const unsigned long TEST_INTERVAL = 4000; // 4 seconds

unsigned long lastTestUpdate = 0;
int testState = 0;

// ====================================
// Setup
// ====================================

void setup() {
    Serial.begin(115200);

    // Initialize motors
    dcMotor.begin();
    stepperMotor.begin();
    servoMotor.attach(SERVO_PIN);

    // DC motor PI gains
    dcMotor.setPIDGains(1.0, 1.5);

    // Stepper speed (microsteps/second)
    stepperMotor.setSpeed(400);

    // Initial positions
    dcMotor.setTargetRPM(0);
    stepperMotor.setTargetDegrees(0);
    servoMotor.write(90);

    lastTestUpdate = millis();

    Serial.println("Motor test initialized");
}

// ====================================
// Test sequence
// ====================================

void runMotorTest() {

    switch (testState) {

        case 0:
            Serial.println("Forward / Stepper +90 / Servo 0");

            dcMotor.setTargetRPM(50);
            stepperMotor.setTargetDegrees(90);
            servoMotor.write(0);
            break;

        case 1:
            Serial.println("Stop / Stepper 0 / Servo 90");

            dcMotor.setTargetRPM(0);
            stepperMotor.setTargetDegrees(0);
            servoMotor.write(90);
            break;

        case 2:
            Serial.println("Reverse / Stepper -90 / Servo 180");

            dcMotor.setTargetRPM(-50);
            stepperMotor.setTargetDegrees(-90);
            servoMotor.write(180);
            break;

        case 3:
            Serial.println("Stop / Stepper 0 / Servo 90");

            dcMotor.setTargetRPM(0);
            stepperMotor.setTargetDegrees(0);
            servoMotor.write(90);
            break;
    }

    // Advance to next test state
    testState = (testState + 1) % 4;
}

// ====================================
// Main loop
// ====================================

void loop() {

    // Continuously update both controllers
    dcMotor.update();
    stepperMotor.update();

    // Change test state every 4 seconds
    if (millis() - lastTestUpdate >= TEST_INTERVAL) {
        lastTestUpdate = millis();

        runMotorTest();
    }
}