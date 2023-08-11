import time
from subprocess import Popen
import os
from random import randrange
import time
import sys

import sys
import termios
import contextlib


current_dir = os.getcwd()

c1 = Popen(f'python3 {current_dir}/Master.py', shell=True)
time.sleep(0.1)
Popen(f'python3 {current_dir}/main.py 1', shell=True)
time.sleep(0.1)
Popen(f'python3 {current_dir}/main.py 2', shell=True)
time.sleep(0.1)
Popen(f'python3 {current_dir}/main.py 3', shell=True)
time.sleep(0.1)
Popen(f'python3 {current_dir}/DB_update_node.py', shell=True)
time.sleep(0.1)
Popen(f'python3 {current_dir}/DB_query_node.py', shell=True)



@contextlib.contextmanager
def raw_mode(file):
    old_attrs = termios.tcgetattr(file.fileno())
    new_attrs = old_attrs[:]
    new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
    try:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, new_attrs)
        yield
    finally:
        termios.tcsetattr(file.fileno(), termios.TCSADRAIN, old_attrs)



def move(ch):
    pass


def main():
    with raw_mode(sys.stdin):
        try:
            while True:
                ch = sys.stdin.read(1)
                print(c1.stdout)
                if not ch or ch == 'c':
                    Popen("kill -9 $(pgrep -f 'Projects/GP')", shell=True)
                    break
                print(ch)
                time.sleep(0.5)

        except (KeyboardInterrupt, EOFError):
            pass


if __name__ == '__main__':
    main()