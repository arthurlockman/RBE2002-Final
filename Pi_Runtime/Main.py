import atexit
from time import sleep
from Arduino import *
from multiprocessing import Process, Pipe

ARDUINO = Arduino()
MOUSE = file('/dev/input/mouse0')


def read_mouse(mouse, conn):
    posx = 0
    posy = 0
    while True:
        status, dx, dy = tuple(ord(c) for c in mouse.read(3))

        def to_signed(n):
            return n - ((0x80 & n) << 1)

        dx = to_signed(dx)
        dy = to_signed(dy)
        posx += dx
        posy += dy
        conn.send([posx, posy])


def kill_daemons():
    mouse_d.terminate()

atexit.register(close_serial_port)
atexit.register(kill_daemons)

if __name__ == '__main__':
    main_conn, mouse_conn = Pipe()
    mouse_d = Process(target=read_mouse, args=(MOUSE, mouse_conn, ))
    mouse_d.start()
    while True:
        print main_conn.recv()
