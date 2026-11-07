const int FLEX_PIN = A0;
const float FIXED_RESISTOR = 10000.0; // 10 kΩ

void setup() {
  Serial.begin(115200);
}

void loop() {
  int adc = analogRead(FLEX_PIN);

  if (adc > 0) {
    // Calculate flex sensor resistance in kΩ
    float resistance = FIXED_RESISTOR * (1023.0 / adc - 1.0);
    float resistanceKOhm = resistance / 1000.0;

    Serial.println(resistanceKOhm, 2);
  }

  delay(100);
}