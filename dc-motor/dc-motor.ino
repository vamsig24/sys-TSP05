// Motor driver pins
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Encoder pins
const int ENCODER_A = 2;
const int ENCODER_B = 3;

// Encoder resolution
const float PULSES_PER_REV = 210.0;

// Encoder counter
volatile long encoderCount = 0;

// Timing
const unsigned long SAMPLE_TIME = 200;
unsigned long lastUpdate = 0;

// Encoder interrupt
void encoderISR() {
  if (digitalRead(ENCODER_B) == LOW) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    encoderISR,
    RISING
  );

  // Command positive direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  // Fixed PWM, no closed-loop control
  analogWrite(ENA, 100);

  lastUpdate = millis();

  Serial.println("MeasuredRPM");
}

void loop() {
  unsigned long now = millis();

  if (now - lastUpdate >= SAMPLE_TIME) {
    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    noInterrupts();
    long pulses = encoderCount;
    encoderCount = 0;
    interrupts();

    float measuredRPM =
      (pulses / PULSES_PER_REV) * (60.0 / dt);

    Serial.println(measuredRPM);
  }
}