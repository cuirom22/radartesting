#ifndef MOTOR_H
#define MOTOR_H


#include <Arduino.h>
#include "config.h"

class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t inA, uint8_t inB, uint8_t enPin, uint8_t channel);
    void begin();
    void setSpeed(int duty);   // 0–1023
    void forward(int duty);
    void reverse(int duty);
    void stop();

private:
    uint8_t _pwm, _inA, _inB, _en, _ch;
};

#endif