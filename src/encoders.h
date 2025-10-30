#ifndef encoders.h
#define encoders.h

#include <Arduino.h>
#include "config.h"

class Encoder {
public: 
    
    Encoder (uint8_t pinA, uint8_t pinB ); // Constructor
    void begin ();
    void update ();
    long getCount();
    void resetCount();
    float getRPM();
private:
    uint8_t _pinA, _pinB;
    volatile long _count;
    unsigned long _lastTime;
    long _lastCount;

    static void handleLeftInterrupt();
    static void handleRightInterrupt();

};

extern Encoder* leftEncoderInstance;
extern Encoder* rightEncoderInstance;



#endif
