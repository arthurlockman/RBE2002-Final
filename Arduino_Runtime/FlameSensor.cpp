#include "FlameSensor.h"

FlameSensor::FlameSensor(int port):
    m_port(port)
{
    //Init complete.
}

int FlameSensor::read()
{
    return analogRead(m_port);
}
