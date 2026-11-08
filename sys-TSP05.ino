#include <Servo.h>
#include "DcMotor.h"
#include "StepperMotor.h"
#include "DistanceSensor.h"
#include "FlexSensor.h"
#include "SharpIRSensor.h"
#include "Potentiometer.h"
#include "Button.h"

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

// VL53L1X distance sensor: I2C (Uno: SDA = A4, SCL = A5)
DistanceSensor distanceSensor;

// Flex sensor (variable resistor): 5V -- flex -- A0 -- 10k -- GND
FlexSensor flexSensor(A0, 10000.0f);

// Sharp GP2Y0A21YK0F IR distance sensor (10-80 cm), analog output
// (A0 is taken by the flex sensor)
SharpIRSensor sharpSensor(A1);

// Potentiometer wiper on A2 (outer legs to 5V and GND)
Potentiometer potentiometer(A2);

// Push button on pin 2: button to 5V, external pull-down (pressed = HIGH).
// NOTE: pin 2 is also the DC motor encoder A interrupt pin.
Button button(2);

// ====================================
// Test configuration
// ====================================

const unsigned long TEST_INTERVAL = 4000; // 4 seconds

unsigned long lastTestUpdate = 0;
int testState = 0;

// Sensor serial output
const unsigned long SENSOR_PRINT_INTERVAL = 200; // ms

unsigned long lastSensorPrint = 0;

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

    // Initialize flex sensor filter
    flexSensor.begin();

    // Initialize Sharp IR sensor
    sharpSensor.begin();

    // Initialize potentiometer filter
    potentiometer.begin();

    // Initialize push button (after dcMotor.begin(), polled, no interrupt)
    button.begin();

    // Initialize distance sensor (motors keep running if it is missing)
    if (distanceSensor.begin()) {
        Serial.println("VL53L1X initialized");
    } else {
        Serial.println("ERROR: VL53L1X not detected! Check wiring.");
    }

    lastTestUpdate = millis();
    lastSensorPrint = millis();

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
// Sensor output
// ====================================

void printSensors() {

    Serial.print("Distance: ");
    if (distanceSensor.isOk()) {
        Serial.print(distanceSensor.getDistanceMm());
        Serial.print(" mm | ");
        Serial.print(distanceSensor.getDistanceCm(), 1);
        Serial.print(" cm");
    } else {
        Serial.print("ERROR");
    }

    Serial.print(" | Flex adc: ");
    Serial.print(flexSensor.getADC(), 1);
    Serial.print(" ");
    Serial.print(flexSensor.readResistance(), 2);
    Serial.print(" kOhm ");
    Serial.print(flexSensor.getBendAngle());
    Serial.print(" deg");

    Serial.print(" | Sharp: ");
    Serial.print(sharpSensor.getVoltage(), 2);
    Serial.print(" V ");
    if (sharpSensor.isValid()) {
        Serial.print(sharpSensor.getDistanceCm());
        Serial.print(" cm");
    } else {
        Serial.print(SharpIRSensor::statusToString(sharpSensor.getStatus()));
    }

    Serial.print(" | Pot: ");
    Serial.print(potentiometer.getVoltage(), 2);
    Serial.print(" V ");
    Serial.print(potentiometer.getPositionPercent());
    Serial.print("%");

    Serial.print(" | Button: ");
    Serial.print(button.isPressed() ? "PRESSED" : "released");
    Serial.print(" (");
    Serial.print(button.getPressCount());
    Serial.print(")");

    Serial.print(" | RPM: ");
    Serial.print(dcMotor.getRPM(), 1);
    Serial.println();
}

// ====================================
// Main loop
// ====================================

void loop() {

    // Continuously update both controllers
    dcMotor.update();
    stepperMotor.update();
    distanceSensor.update();
    flexSensor.update();
    sharpSensor.update();
    potentiometer.update();
    button.update();

    if (button.wasPressed()) {
        Serial.println("Button pressed");
    }

    // Print sensor readings periodically
    if (millis() - lastSensorPrint >= SENSOR_PRINT_INTERVAL) {
        lastSensorPrint = millis();

        printSensors();
    }

    // Change test state every 4 seconds
    if (millis() - lastTestUpdate >= TEST_INTERVAL) {
        lastTestUpdate = millis();

        runMotorTest();
    }
}