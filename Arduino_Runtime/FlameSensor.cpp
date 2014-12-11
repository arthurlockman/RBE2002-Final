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
	int y = this->read();
	if (y < K_CUTOFF)
	{
		double base = K_C * ((y - K_A) / (K_D - y));
		double power = 1 / K_B;
		return pow(base, power);
	}
	else { return -1.0; }
}

float FlameSensor::height()
{
	int x = this->read();
	return 0.0;
}