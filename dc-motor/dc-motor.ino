const int ENA = 9;  // PWM speed control
const int IN1 = 8;  // Direction
const int IN2 = 7;  // Direction

int motorSpeed = 150;  // Speed from 0 to 255

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Set motor direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Set motor speed
  analogWrite(ENA, motorSpeed);
}

void loop() {
  // Nothing needed here
}