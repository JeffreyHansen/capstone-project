import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc

print('''
**********************************************************
****Function: Hiwonder Raspberry Pi Expansion Board, Bus Servo Speed Control****
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If it fails, try multiple times!
----------------------------------------------------------
''')
board = rrc.Board()
start = True

# Pre-shutdown handling
def Stop(signum, frame):
    global start
    start = False
    print('Closing...')

signal.signal(signal.SIGINT, Stop)

if __name__ == '__main__':
    while True:
        board.bus_servo_set_position(0.5, [[1, 0],[2, 0]])
        time.sleep(0.5)
        board.bus_servo_set_position(2, [[1, 1000], [2, 1000]])
        time.sleep(1)
        board.bus_servo_stop([1, 2])
        time.sleep(1)
        if not start:
            board.bus_servo_stop([1, 2])
            time.sleep(1)
            print('Closed')
            break