from serial import Serial
import atexit
from time import sleep

ARDUINO_COMM = Serial("/dev/ttyACM0", 115200)


def close_serial_port():
    """Closes a serial port at program exit."""
    ARDUINO_COMM.close()
    print "Closing serial port..."


atexit.register(close_serial_port)
