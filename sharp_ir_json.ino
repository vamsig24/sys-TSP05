#include <SharpIR.h>

#define IRPin A0
#define model 1080

#define MIN_DISTANCE 10
#define MAX_DISTANCE 80

SharpIR mySensor = SharpIR(IRPin, model);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  int distance_cm = mySensor.distance();

  bool valid = (distance_cm >= MIN_DISTANCE &&
                distance_cm <= MAX_DISTANCE);

  const char* status;

  if (distance_cm <= 0)
    status = "INVALID_READING";
  else if (distance_cm < MIN_DISTANCE)
    status = "TOO_CLOSE";
  else if (distance_cm > MAX_DISTANCE)
    status = "TOO_FAR";
  else
    status = "VALID";

  Serial.print("{\"type\":\"state\",\"sharp_distance_cm\":");

  if (valid)
    Serial.print(distance_cm);
  else
    Serial.print("null");

  Serial.print(",\"sharp_valid\":");
  Serial.print(valid ? "true" : "false");

  Serial.print(",\"sharp_status\":\"");
  Serial.print(status);
  Serial.println("\"}");

  delay(200);
}
