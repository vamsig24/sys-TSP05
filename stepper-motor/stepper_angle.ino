#include <math.h>

const byte DIR_PIN  = 12;
const byte STEP_PIN = 13;

// Mercury motor: 200 full steps/revolution.
// EasyDriver defaults to 1/8 microstepping.
const long MICROSTEPS_PER_REVOLUTION = 1600;

const unsigned int STEP_DELAY_US = 1500;

void moveDegrees(float degrees) {
  if (degrees == 0.0) {
    return;
  }

  digitalWrite(DIR_PIN, degrees > 0 ? HIGH : LOW);
  delayMicroseconds(10);

  long numberOfSteps = lround(
    fabs(degrees) * MICROSTEPS_PER_REVOLUTION / 360.0
  );

  Serial.print("Moving ");
  Serial.print(degrees, 2);
  Serial.print(" degrees using ");
  Serial.print(numberOfSteps);
  Serial.println(" microsteps.");

  for (long i = 0; i < numberOfSteps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }

  Serial.println("Movement complete.");
}

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);

  Serial.begin(9600);
  Serial.setTimeout(100);

  Serial.println("Enter an angle in degrees.");
  Serial.println("Positive: one direction");
  Serial.println("Negative: opposite direction");
}

void loop() {
  if (Serial.available() > 0) {
    float requestedDegrees = Serial.parseFloat();

    while (Serial.available() > 0) {
      Serial.read();
    }

    moveDegrees(requestedDegrees);
    Serial.println("Enter another angle:");
  }
}