#ifndef SINGLEENCODER_H
#define SINGLEENCODER_H

#include <Arduino.h>

class SingleEncoder
{
public:
	SingleEncoder(int pin, int ticksPerRev);
	uint32_t read();
	float speed();
	//1 is forwards, 0 is backwards
	void update(int dir);
	void write(uint32_t newPosition);
	int interruptPin;
private:
	uint32_t position;
	long startTime;
	int counter;
	float m_speed;
	int m_ticksPerRev;
};
#endif
