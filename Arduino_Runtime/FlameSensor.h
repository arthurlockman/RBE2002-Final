#ifndef FLAMESENSOR_H
#define FLAMESENSOR_H

#include "Arduino.h"

#define K_A 23.92856
#define K_B 7.783453
#define K_C 25.06772
#define K_D 849.9155
#define K_CUTOFF 1000

class FlameSensor
{
public:
    FlameSensor(int port);
    int read();
    int rawRead();
    float distance();
    float height();
private:
    int m_port;    
};

#endif
