
#define POT_PIN A0

#define SAMPLE_COUNT 10
#define REPORT_INTERVAL 100

// ADC reference voltage (nominal)
#define ADC_REFERENCE 5.0

int rawADC = 0;
float filteredADC = 0;

int samples[SAMPLE_COUNT];
int sampleIndex = 0;

unsigned long previousMillis = 0;

void setup()
{
  Serial.begin(9600);

  // Initialize moving average buffer
  long total = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++)
  {
    samples[i] = analogRead(POT_PIN);
    total += samples[i];
  }

  filteredADC = (float)total / SAMPLE_COUNT;

  Serial.println("======================================");
  Serial.println("     ARDUINO POTENTIOMETER TEST");
  Serial.println("======================================");
  Serial.println("Raw ADC | Filtered ADC | Raw Voltage | Filtered Voltage | Position");
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= REPORT_INTERVAL)
  {
    previousMillis = currentMillis;

    // Read raw ADC value
    rawADC = analogRead(POT_PIN);

    // Update moving average buffer
    samples[sampleIndex] = rawADC;

    sampleIndex++;

    if (sampleIndex >= SAMPLE_COUNT)
    {
      sampleIndex = 0;
    }

    // Calculate filtered ADC
    long total = 0;

    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
      total += samples[i];
    }

    filteredADC = (float)total / SAMPLE_COUNT;

    // Convert ADC readings into voltage
    float rawVoltage =
      rawADC * (ADC_REFERENCE / 1023.0);

    float filteredVoltage =
      filteredADC * (ADC_REFERENCE / 1023.0);

    // Calculate position percentage
    int position = map(
      (int)(filteredADC + 0.5),
      0, 1023,
      0, 100
    );

    // Display results
    Serial.print(rawADC);
    Serial.print("       | ");

    Serial.print(filteredADC, 1);
    Serial.print("        | ");

    Serial.print(rawVoltage, 2);
    Serial.print(" V      | ");

    Serial.print(filteredVoltage, 2);
    Serial.print(" V           | ");

    Serial.print(position);
    Serial.println("%");
  }
}
