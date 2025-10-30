#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include "encoders.h"
#include "config.h"

class Odometry {
public:
    Odometry(Encoder* leftEnc, Encoder* rightEnc);
    void update();           // call every loop
    float getX();            // cm
    float getY();            // cm
    float getTheta();        // radians

private:
    Encoder* _leftEnc;
    Encoder* _rightEnc;
    long _prevLeft;
    long _prevRight;
    float _x, _y, _theta;
};

#endif