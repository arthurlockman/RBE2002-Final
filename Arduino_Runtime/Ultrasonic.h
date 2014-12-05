#ifndef Ultrasonic_h
#define Ultrasonic_h

#include "Arduino.h"

class Ultrasonic
{
public:
    Ultrasonic(int trig, int echo);
    int rawRead();
    void activate();
private:
    int m_trig;
    int m_echo;
};

#endif
