/*
  Ultrasonic.h - Library for finding the distance to an object in inches.
*/

#ifndef Ultrasonic_h
#define Ultrasonic_h

#include "Arduino.h"

class Ultrasonic {
public:
  Ultrasonic(int pingPin);
  float distance();
  float calc();
private:
  int _pingPin;
};

#endif

