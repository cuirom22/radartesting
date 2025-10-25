#include <Arduino.h>
#include "config.h"
#include "ultrasonic.h"

float distances[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  ultrasonicInit();
  Serial.println("Ultrasonic system ready.");
}

void loop() {
  scanAllUltrasonics(distances);

  Serial.print("Distances (cm): ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(distances[i]);
    Serial.print("\t");
  }
  Serial.println();

  delay(100); // ~5 Hz overall scan rate
}