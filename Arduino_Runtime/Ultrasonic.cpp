#include "Arduino.h"
#include "Ultrasonic.h"

Ultrasonic::Ultrasonic(int trig, int echo):
	m_trig(trig),
	m_echo(echo)
{

}

int Ultrasonic::rawRead()
{
	return analogRead(m_echo);
}

void Ultrasonic::activate()
{
	pinMode(m_trig, OUTPUT);
	pinMode(m_echo, INPUT);
	digitalWrite(m_trig, HIGH);
}
