#include "Motor.h"

Motor::Motor(uint8_t pwmPin, uint8_t inA, uint8_t inB, uint8_t enPin, uint8_t channel)
    : _pwm(pwmPin), _inA(inA), _inB(inB), _en(enPin), _ch(channel) {}

void Motor::begin() {
    pinMode(_inA, OUTPUT);
    pinMode(_inB, OUTPUT);
    pinMode(_en, OUTPUT);
    digitalWrite(_en, HIGH);

    ledcSetup(_ch, PWM_FREQ, PWM_RES);
    ledcAttachPin(_pwm, _ch);
}

void Motor::setSpeed(int duty) {
    ledcWrite(_ch, constrain(duty, 0, 1023));
}

void Motor::forward(int duty) {
    digitalWrite(_inA, HIGH);
    digitalWrite(_inB, LOW);
    setSpeed(duty);
}

void Motor::reverse(int duty) {
    digitalWrite(_inA, LOW);
    digitalWrite(_inB, HIGH);
    setSpeed(duty);
}

void Motor::stop() {
    digitalWrite(_inA, LOW);
    digitalWrite(_inB, LOW);
    setSpeed(0);
}