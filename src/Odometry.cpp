#include "Odometry.h"

Odometry::Odometry(Encoder* leftEnc, Encoder* rightEnc)
  : _leftEnc(leftEnc), _rightEnc(rightEnc),
    _prevLeft(0), _prevRight(0), _x(0), _y(0), _theta(0), _lastUpdateMs(0), _vCms(0) {}

void Odometry::update() {
    long leftCount  = _leftEnc->getCount();
    long rightCount = _rightEnc->getCount();

    long dLeft  = leftCount  - _prevLeft;
    long dRight = rightCount - _prevRight;

    _prevLeft  = leftCount;
    _prevRight = rightCount;

    // distance per tick
    float distPerTick = WHEEL_CIRCUM_CM / ENCODER_RESOLUTION;
    float leftDist  = dLeft  * distPerTick;
    float rightDist = dRight * distPerTick;
    float deltaDist = (leftDist + rightDist) / 2.0;
    float deltaTheta = (rightDist - leftDist) / WHEEL_BASE_CM;

    // update pose
    _x     += deltaDist * cos(_theta + deltaTheta / 2.0);
    _y     += deltaDist * sin(_theta + deltaTheta / 2.0);
    _theta += deltaTheta;

    unsigned long now = millis();
    if (_lastUpdateMs == 0) _lastUpdateMs = now;
    float dt = (now - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = now;

    if (dt > 0) _vCms = deltaDist / dt;


}

float Odometry::getX()     { return _x; }
float Odometry::getY()     { return _y; }
float Odometry::getTheta() { return _theta; }
float Odometry::getSpeedCms() { return _vCms; }
