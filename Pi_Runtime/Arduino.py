from serial import Serial
import sys

if sys.platform.startswith('darwin'):
    ARDUINO_COMM = Serial('/dev/tty.usbmodem1411', 115200)
elif sys.platform.startswith('linux2'):
    ARDUINO_COMM = Serial("/dev/ttyACM0", 115200)
elif sys.platform.startswith('win32'):
    ARDUINO_COMM = Serial("COM1", 115200)


def close_serial_port():
    """Closes a serial port at program exit."""
    ARDUINO_COMM.close()
    print "Closing serial port..."


class Arduino:
    """A class for communicating with the Arduino over serial."""
    def __init__(self):
        pass

    def send(self, message):
        """Send a message over the serial channel. Will be automatically terminated with a newline."""
        ARDUINO_COMM.flush()
        ARDUINO_COMM.write(message + '\\n')
