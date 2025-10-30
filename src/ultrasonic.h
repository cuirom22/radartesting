#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>

class Ultrasonic {
public:
    Ultrasonic(uint8_t trig, uint8_t echo);
    void begin();
    float getDistanceCM();

private:
    uint8_t _trig, _echo;
};

#endif