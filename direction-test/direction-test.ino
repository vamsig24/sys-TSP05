// ====================================
// Pin configuration
// ====================================

// L298N motor driver
const int ENA = 9;
const int IN1 = 8;
const int IN2 = 7;

// Quadrature encoder
const int ENCODER_A = 2;
const int ENCODER_B = 3;

// ====================================
// Motor configuration
// ====================================

// 7 pulses per motor revolution
// 30:1 gearbox
// Counting rising edges on channel A
const float PULSES_PER_REV = 210.0;

// Desired speed in RPM
// Positive = forward
// Negative = reverse
float targetRPM = 0.0;

// ====================================
// PI controller
// ====================================

float Kp = 1.0;
float Ki = 1.5;

float integral = 0.0;
float measuredRPM = 0.0;

// PWM limits
const int MAX_PWM = 230;

// Limit change in PWM per control cycle
// Helps prevent sudden reversals
const int MAX_PWM_CHANGE = 30;

int currentPWM = 0;

// ====================================
// Encoder
// ====================================

volatile long encoderCount = 0;

// Encoder direction corrected based on your test
void encoderISR() {
  if (digitalRead(ENCODER_B) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

// ====================================
// Timing
// ====================================

const unsigned long SAMPLE_TIME = 200; // ms
unsigned long lastUpdate = 0;

// ====================================
// Motor control
// ====================================

void setMotor(int pwm) {

  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  // Limit PWM changes to avoid sudden reversals
  int change = constrain(
    pwm - currentPWM,
    -MAX_PWM_CHANGE,
    MAX_PWM_CHANGE
  );

  currentPWM += change;

  if (currentPWM > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

  } else if (currentPWM < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, abs(currentPWM));
}

// ====================================
// Serial input
// ====================================

void readSerialCommand() {

  if (Serial.available() > 0) {

    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
      return;
    }

    float rpm = input.toFloat();

    if (rpm >= -100.0 && rpm <= 100.0) {

      targetRPM = rpm;

      // Reset integral when changing target
      integral = 0.0;

      Serial.print("New target RPM: ");
      Serial.println(targetRPM);

    } else {
      Serial.println("Error: Enter RPM between -100 and 100");
    }
  }
}

// ====================================
// Setup
// ====================================

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(50);

  // Motor driver
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Initially stopped
  analogWrite(ENA, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // Encoder interrupt
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_A),
    encoderISR,
    RISING
  );

  lastUpdate = millis();

  Serial.println("Bidirectional PI Speed Controller");
  Serial.println("Enter target RPM (-100 to 100):");
  Serial.println("TargetRPM,MeasuredRPM,PWM");
}

// ====================================
// Main loop
// ====================================

void loop() {

  // Check for new speed commands
  readSerialCommand();

  unsigned long now = millis();

  if (now - lastUpdate >= SAMPLE_TIME) {

    float dt = (now - lastUpdate) / 1000.0;
    lastUpdate = now;

    // --------------------------------
    // Read encoder
    // --------------------------------

    noInterrupts();
    long pulses = encoderCount;
    encoderCount = 0;
    interrupts();

    // --------------------------------
    // Calculate signed RPM
    // --------------------------------

    measuredRPM =
      (pulses / PULSES_PER_REV) *
      (60.0 / dt);

    // --------------------------------
    // PI controller
    // --------------------------------

    float error = targetRPM - measuredRPM;

    if (targetRPM == 0.0) {

      integral = 0.0;
      setMotor(0);

    } else {

      // Integrate error
      integral += error * dt;

      // Limit integral windup
      integral = constrain(integral, -150.0, 150.0);

      // Calculate PI output
      float output = Kp * error + Ki * integral;

      // Apply motor command
      setMotor((int)output);
    }

    // --------------------------------
    // Serial monitoring
    // --------------------------------

    Serial.print(targetRPM);
    Serial.print(",");
    Serial.print(measuredRPM);
    Serial.print(",");
    Serial.println(currentPWM);
  }
}