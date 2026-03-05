#!/usr/bin/python3
# coding=utf8
import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
****Function: Hiwonder Raspberry Pi Expansion Board, Multiple PWM Servo Control****
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
        board.pwm_servo_set_position(1, [[1, 1100]]) # Set servo 1 pulse width to 1100, run time 1000ms
        time.sleep(1)
        board.pwm_servo_set_position(1, [[1, 1500]]) # Set servo 1 pulse width to 1500, run time 1000ms
        time.sleep(1)
        board.pwm_servo_set_position(1, [[1, 1900], [2, 1000]]) # Set servo 1 pulse width to 1900, servo 2 to 1000, run time 1000ms
        time.sleep(1)
        board.pwm_servo_set_position(1, [[1, 1500], [2, 2000]]) # Set servo 1 pulse width to 1500, servo 2 to 2000, run time 1000ms
        time.sleep(1)       
        if not start:
            board.pwm_servo_set_position(1, [[1, 1500], [2, 1500]]) # Set servo 1 pulse width to 1500, servo 2 to 1500, run time 1000ms
            time.sleep(1)
            print('Closed')
            break
    
    
        
