#include "SingleEncoder.h"

SingleEncoder::SingleEncoder(int pin, int ticksPerRev) :
	interruptPin(0),
	m_ticksPerRev(ticksPerRev)
{
	switch (pin)
	{
		case 2:  interruptPin = 0; break;
		case 3:  interruptPin = 1; break;
		case 21: interruptPin = 2; break;
		case 20: interruptPin = 3; break;
		case 19: interruptPin = 4; break;
		case 18: interruptPin = 5; break;
	}
}

uint32_t SingleEncoder::read()
{
	return position;
}

void SingleEncoder::update(int dir)
{
	if (dir == 1)
		position++;
	else if (dir == 0)
		position--;

	if (counter == 20)
	{
		counter = 0;
		long dT = millis() - startTime;
		startTime = millis();
		m_speed = (1.0 / ((float)dT * (float)m_ticksPerRev)) * 1200000.0;
		if (dir == 0) m_speed = -m_speed;
	}
	counter++;
}

void SingleEncoder::write(uint32_t newPosition)
{
	position = newPosition;
}

float SingleEncoder::speed()
{
	return m_speed;
}

float SingleEncoder::distance()
{
	return ((float)position / (float)m_ticksPerRev) * 2.75 * M_PI;
}
