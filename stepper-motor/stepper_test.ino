const byte DIR_PIN  = 2;
const byte STEP_PIN = 3;

void rotateOneTurn(bool direction) {
  digitalWrite(DIR_PIN, direction);
  delay(5);

  // 200 full steps × 8 microsteps = 1600 pulses
  for (int i = 0; i < 1600; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(1500);
  }
}

void setup() {
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  delay(1000);
}

void loop() {
  rotateOneTurn(HIGH);
  delay(1000);

  rotateOneTurn(LOW);
  delay(1000);
}