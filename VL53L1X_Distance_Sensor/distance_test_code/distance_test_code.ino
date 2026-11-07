
#include <Wire.h>
#include <SparkFun_VL53L1X.h>

SFEVL53L1X distanceSensor;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Initializing VL53L1X...");

  // Initialize the sensor
  if (distanceSensor.begin() != 0) {
    Serial.println("ERROR: VL53L1X not detected!");
    Serial.println("Check wiring.");
    while (1);
  }

  Serial.println("VL53L1X initialized successfully!");

  // Long distance mode
  distanceSensor.setDistanceModeLong();

  // Set measurement interval to 100 ms
  distanceSensor.setTimingBudgetInMs(100);

  // Start measuring
  distanceSensor.startRanging();
}

void loop() {
  // Wait for measurement to be ready
  unsigned long startTime = millis();

  while (!distanceSensor.checkForDataReady()) {
    if (millis() - startTime > 1000) {
      Serial.println("ERROR: Sensor timeout!");
      delay(500);
      return;
    }
    delay(1);
  }

  // Read distance in millimeters
  int distance_mm = distanceSensor.getDistance();

  // Clear interrupt for next reading
  distanceSensor.clearInterrupt();

  // Convert to centimeters
  float distance_cm = distance_mm / 10.0;

  Serial.print("Distance: ");
  Serial.print(distance_mm);
  Serial.print(" mm | ");
  Serial.print(distance_cm, 1);
  Serial.println(" cm");

  delay(100);
}
