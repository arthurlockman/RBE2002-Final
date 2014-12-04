#ifndef FLAMESENSOR_H
#define FLAMESENSOR_H

#include "Arduino.h"

class FlameSensor
{
public:
    FlameSensor(int port);
    int read();
private:
    int m_port;    
};

#endif
