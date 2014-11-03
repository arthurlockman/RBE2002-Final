import atexit
from time import sleep
from Arduino import *
from multiprocessing import Process, Pipe

ARDUINO = Arduino()
MOUSE = file('/dev/input/mouse0')


def mouse_daemon(mouse, conn):
    """A daemon for reading mouse position in the background."""
    pos_x = long(0)
    pos_y = long(0)
    while True:
        status, dx, dy = tuple(ord(c) for c in mouse.read(3))

        def to_signed(n):
            return n - ((0x80 & n) << 1)

        dx = to_signed(dx)
        dy = to_signed(dy)
        pos_x += dx
        pos_y += dy
        conn.send([pos_x, pos_y])


def kill_daemons():
    mouse_d.terminate()

atexit.register(close_serial_port)
atexit.register(kill_daemons)

if __name__ == '__main__':
    main_conn, mouse_conn = Pipe()
    mouse_d = Process(target=mouse_daemon, args=(MOUSE, mouse_conn, ))
    mouse_d.start()
    while True:
        print main_conn.recv()
