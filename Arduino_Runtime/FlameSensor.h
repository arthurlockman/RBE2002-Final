#ifndef FLAMESENSOR_H
#define FLAMESENSOR_H

#include "Arduino.h"

#define K_A 22.27373914
#define K_B 7.436106118
#define K_C 25.23852275
#define K_D 867.288791
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
