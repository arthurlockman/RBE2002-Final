#include "FlameSensor.h"

FlameSensor::FlameSensor(int port):
    m_port(port)
{
    //Init complete.
}

int FlameSensor::rawRead()
{
    return analogRead(m_port);
}

int FlameSensor::read()
{
    int sum = 0;
    for (int i = 0; i < 10; i++)
    {
        sum += this->rawRead();
    }
    return sum / 10;
}

float FlameSensor::distance()
{
	float y = (float)this->read();
	if (y < K_D)
	{
		double base = ((K_A - K_D) / (y - K_D) - 1);
		double power = (1 / K_B);
		return pow(base, power) * K_C;
	}
	else if (y > K_D && y < 1000) { return -1.0; }
	else { return -2.0; }
}

float FlameSensor::height()
{
	int x = this->read();
	return 0.0;
}