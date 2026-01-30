#include "encoders.h"

Encoder* leftEncoderInstance = nullptr;
Encoder* rightEncoderInstance = nullptr;

Encoder::Encoder(uint8_t pinA, uint8_t pinB)
    : _pinA(pinA), _pinB(pinB), _count(0), _lastTime(0), _lastCount(0) {}

void Encoder::begin() {
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);

  // Identify which encoder based on pinA
  if (_pinA == L_ENCA) {
    leftEncoderInstance = this;
    attachInterrupt(digitalPinToInterrupt(_pinA), handleLeftInterrupt, CHANGE);
  }

  if (_pinA == R_ENCA) {
    rightEncoderInstance = this;
    attachInterrupt(digitalPinToInterrupt(_pinA), handleRightInterrupt, CHANGE);
  }
}


void Encoder::handleLeftInterrupt() {
    if (leftEncoderInstance != nullptr) leftEncoderInstance->update();
}

void Encoder::handleRightInterrupt() {
    if (rightEncoderInstance != nullptr) rightEncoderInstance->update();
}

void Encoder::update() {
    int a = digitalRead(_pinA);
    int b = digitalRead(_pinB);
    if (a==b) _count++;
    else _count--;
//what
}

long Encoder::getCount() {
    return _count;
}

void Encoder::resetCount() {
    _count = 0;
}

float Encoder:: getRPM() {
    unsigned long now = millis();
    long countNow = _count;
    float deltaT = (now - _lastTime) / 1000.0;
    long deltaCount = countNow - _lastCount;

    _lastTime = now;
    _lastCount = countNow;

    float revs = deltaCount /36.0; // 36 teeth per rev
    return (revs/ deltaT) * 60.0;
}