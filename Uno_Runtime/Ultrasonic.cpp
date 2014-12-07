/*
  Ultrasonic.cpp - Library for finding the distance to an object in inches.
  Takes 4 samples and averages them together for a better reading.
 */

#include "Arduino.h"
#include "Ultrasonic.h"


Ultrasonic::Ultrasonic(int pingPin, int echoPin) {
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);

  _pingPin = pingPin;
  _echoPin = echoPin;
}

float Ultrasonic::calc() {
//pulse
  //make sure pin is low
  digitalWrite(_pingPin, LOW);
  //pull high to start the pulse
  digitalWrite(_pingPin, HIGH);
  //250us pulse according to VEX datasheets
  delayMicroseconds(250);
  //pull back low
  digitalWrite(_pingPin, LOW);

  //measure the length of the high pulse
  //goes low when the ping returns
  long pulses = pulseIn(_echoPin, HIGH);

  //speed_of_sound/2 * time * 12in/ft
  float dist = ( (567.5 * ( (float)pulses/1000000)) * 12.0);

  return dist;

}

float Ultrasonic::distance() {
  float avgDist = 0;
  
  //take 5 samples
  for(int i = 0; i < 5; i++) {
	avgDist += calc();
	//wait 10ms -> otherwist pulseIn times out
	delay(20);
  }
  //divide by 5
  avgDist /= 5;
  
  return avgDist;
}


