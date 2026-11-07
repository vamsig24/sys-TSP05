// Servo test: moves 0 -> 90 -> 180 -> 90 -> 0, pausing 1 s at each step
#include <Servo.h>

const int SERVO_PIN = 10;
const int angles[] = {0, 90, 180, 90};

Servo servo;

void setup() {
  Serial.begin(115200);
  servo.attach(SERVO_PIN);
}

void loop() {
  for (int angle : angles) {
    servo.write(angle);
    Serial.print("servo:");
    Serial.println(angle);
    delay(1000);
  }
}
