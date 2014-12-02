#include "LightSensor.h"
#include "Arduino.h"

LightSensor::LightSensor(int pin, int threshold, int mode):
	m_threshold(threshold),
	m_pin(pin),
	m_mode(mode)
{

}

int LightSensor::read()
{
	if (m_mode == 0)
	{
		if (analogRead(m_pin) >= m_threshold) return 1;
		else return 0;
	} else {
		if (analogRead(m_pin) <= m_threshold) return 1;
		else return 0;
	}
}

int LightSensor::rawRead()
{
	return analogRead(m_pin);
}
