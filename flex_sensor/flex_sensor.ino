// Flex sensor on A0: 5V -- flex sensor -- A0 -- 10k -- GND
// Bend with the printed side on the outside of the curve
// Futaba servo signal on pin 10
// Servo angle is mapped from sensor resistance: straight = 0, max bend = 180

#include <Servo.h>

const int FLEX_PIN = A0;
const int SERVO_PIN = 10;

const float R_FIXED = 10000.0;  // fixed divider resistor
const float R_FLAT = 13000.0;   // sensor resistance when straight
const float R_BENT = 34000.0;   // sensor resistance at max bend

const float EMA_ALPHA = 0.15;   // lower = smoother
const int DEADZONE_DEG = 10;    // servo stays at 0 below this
const int DEADBAND_DEG = 5;     // ignore servo changes smaller than this

const int MEDIAN_SIZE = 5;
int samples[MEDIAN_SIZE];
int sampleIdx = 0;
float smoothed = 0;

Servo servo;
int servoAngle = 0;

int median() {
  int sorted[MEDIAN_SIZE];
  memcpy(sorted, samples, sizeof(samples));
  for (int i = 1; i < MEDIAN_SIZE; i++) {
    for (int j = i; j > 0 && sorted[j - 1] > sorted[j]; j--) {
      int t = sorted[j]; sorted[j] = sorted[j - 1]; sorted[j - 1] = t;
    }
  }
  return sorted[MEDIAN_SIZE / 2];
}

float readFiltered() {
  samples[sampleIdx] = analogRead(FLEX_PIN);
  sampleIdx = (sampleIdx + 1) % MEDIAN_SIZE;
  smoothed += EMA_ALPHA * (median() - smoothed);
  return smoothed;
}

float sensorResistance(float adc) {
  adc = constrain(adc, 1, 1022);
  return R_FIXED * (1023.0 / adc - 1.0);
}

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN);
  servo.write(0);

  for (int i = 0; i < MEDIAN_SIZE; i++) samples[i] = analogRead(FLEX_PIN);
  smoothed = analogRead(FLEX_PIN);
}

void loop() {
  float adc = readFiltered();
  float r = sensorResistance(adc);

  int target = (r - R_FLAT) * 180.0 / (R_BENT - R_FLAT);
  target = constrain(target, 0, 180);
  if (target < DEADZONE_DEG) target = 0;

  if (abs(target - servoAngle) >= DEADBAND_DEG || target == 0 || target == 180) {
    servoAngle = target;
    servo.write(servoAngle);
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("adc: ");
    Serial.print(adc, 1);
    Serial.print("  flex: ");
    Serial.print(r / 1000.0, 2);
    Serial.print(" kOhm  -> servo written: ");
    Serial.print(servoAngle);
    Serial.println(" deg");
  }

  delay(20);
}
