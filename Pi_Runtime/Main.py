import serial
import atexit
from time import sleep

ARDUINO_COMM = serial.Serial("/dev/ttyACM0", 115200)

def closeSerialPort():
	ARDUINO_COMM.close()
	print "Closing serial port..."

atexit.register(closeSerialPort)

