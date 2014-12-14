#ifndef SINGLEENCODER_H
#define SINGLEENCODER_H

#include <Arduino.h>

class SingleEncoder
{
public:
	SingleEncoder(int pin, int ticksPerRev);
	int read();
	float speed();
	//1 is forwards, 0 is backwards
	void update(int dir);
	void write(int newPosition);
	int interruptPin;
	float distance();
private:
	int position;
	long startTime;
	int counter;
	float m_speed;
	int m_ticksPerRev;
};
#endif
