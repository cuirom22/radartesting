#include "ultrasonic.h"

// Pin setup (same as your layout diagram)
const int trigPins[NUM_SENSORS] = {21, 22, 23, 2, 8, 9};
const int echoPins[NUM_SENSORS] = {32, 33, 14, 15, 34, 35};

void ultrasonicInit() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
  }
}

float readUltrasonic(int index) {
  if (index < 0 || index >= NUM_SENSORS) return -1;

  // Trigger pulse
  digitalWrite(trigPins[index], LOW);
  delayMicroseconds(2);
  digitalWrite(trigPins[index], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[index], LOW);

  // Measure echo
  long duration = pulseIn(echoPins[index], HIGH, 30000); // 30 ms timeout
  if (duration == 0) return -1; // no echo

  float distance = duration * 0.0343 / 2.0; // cm
  return distance;
}

void scanAllUltrasonics(float distances[]) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    distances[i] = readUltrasonic(i);
    delay(35); // avoid crosstalk
  }
}