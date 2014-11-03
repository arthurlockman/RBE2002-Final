import atexit
from time import sleep
from Arduino import *

ARDUINO = Arduino()

atexit.register(close_serial_port)
