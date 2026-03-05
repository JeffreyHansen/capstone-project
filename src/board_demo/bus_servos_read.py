import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc

print('''
**********************************************************
****Function: Hiwonder Raspberry Pi Expansion Board, Read Bus Servo Data****
**********************************************************
----------------------------------------------------------
Official website: https://www.hiwonder.com
Online mall: https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close this program. If it fails, try multiple times!
----------------------------------------------------------
''')
board = rrc.Board()
board.enable_reception()
start = True

# Pre-shutdown handling
def Stop(signum, frame):
    global start
    start = False
    print('Closing...')

signal.signal(signal.SIGINT, Stop)

def bus_servo_test(board):
    servo_id = board.bus_servo_read_id()
    servo_id = servo_id[0]
    vin =  board.bus_servo_read_vin(servo_id)
    temp = board.bus_servo_read_temp(servo_id)
    position = board.bus_servo_read_position(servo_id)
    # Output servo status
    print("id:", servo_id)
    print('vin:', vin)
    print('temp:',temp)
    print('position',position)


if __name__ == '__main__':
    try:
        while start:
            bus_servo_test(board)
            time.sleep(1)
    except KeyboardInterrupt:
        print('Forced exit')