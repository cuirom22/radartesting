#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(uint8_t trig, uint8_t echo)
    : _trig(trig), _echo(echo) {}

void Ultrasonic::begin() {
    pinMode(_trig, OUTPUT);
    pinMode(_echo, INPUT);
}

// Returns distance in centimeters
float Ultrasonic::getDistanceCM() {
    // Send a 10µs pulse on TRIG
    digitalWrite(_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig, LOW);

    // Measure the length of the ECHO pulse
    long duration = pulseIn(_echo, HIGH, 30000); // 30ms timeout (≈5m max distance)

    // Convert duration (µs) to distance (cm)
    // Sound speed = 343 m/s → 0.0343 cm/µs
    float distance = (duration * 0.0343) / 2.0;

    // Safety: limit invalid readings
    if (distance <= 2 || distance > 400) {
        return -1;  // invalid / out of range
    }
    return distance;
}