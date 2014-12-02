#ifndef LIGHTSENSOR_H
#define LIGHTSENSOR_H

class LightSensor
{
public:
	//Mode 0 uses > threshold, 1 uses < threshold
	LightSensor(int pin, int threshold = 512, int mode = 0);
	int read();
	int rawRead();
private:
	int m_threshold;
	int m_pin;
	int m_mode;
};

#endif
