#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

// Number of sensors
#define NUM_SENSORS 6

// Pin arrays
extern const int trigPins[NUM_SENSORS];
extern const int echoPins[NUM_SENSORS];

// Initialization function
void ultrasonicInit();

// Measure distance (cm) from one sensor
float readUltrasonic(int index);

// Sequentially scan all sensors and fill array
void scanAllUltrasonics(float distances[]);

#endif