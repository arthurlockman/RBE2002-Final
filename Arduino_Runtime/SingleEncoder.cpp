#include "SingleEncoder.h"

SingleEncoder::SingleEncoder(int pin) :
	interruptPin(0)
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
	else if (dir == 0);
		position--;
}

void SingleEncoder::write(uint32_t newPosition)
{
	position = newPosition;
}
