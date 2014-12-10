/*
  Ultrasonic.cpp - Library for finding the distance to an object in inches.
  Takes 4 samples and averages them together for a better reading.
 */

#include "Arduino.h"
#include "Ultrasonic.h"


Ultrasonic::Ultrasonic(int pingPin) {

  _pingPin = pingPin;
}

float Ultrasonic::calc() {
  return (float)analogRead(_pingPin) * (5.0/1023.0)*(1.0/.0098);

}

float Ultrasonic::distance() {
  float avgDist = 0;
  
  //take 5 samples
  for(int i = 0; i < 5; i++) {
	avgDist += calc();
  avgDist += 2.5;
  }
  //divide by 5
  avgDist /= 5;
  
  return avgDist;
}


