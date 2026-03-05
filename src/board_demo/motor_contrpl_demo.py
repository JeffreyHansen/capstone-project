#!/usr/bin/python3
# coding=utf8
import sys
import time
import signal
import ros_robot_controller_sdk as rrc

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
****Function: Hiwonder Raspberry Pi Expansion Board, DC Motor Control****
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
    board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])  # Turn off all motors

signal.signal(signal.SIGINT, Stop)
def Forward(speed):
    for x in range(1,5):
        if x%2 == 0:
            direct = 1
        else:
            direct = -1
        board.set_motor_duty([[x, direct * speed]])

def Turn(speed, direction):
    if (direction == "left"):
        direct = -1
    else:
        direct = 1

    for x in range(3,5): #back wheels
        board.set_motor_duty([[ x , direct * speed]]) 
    for i in range(1,3): #front wheels
        board.set_motor_duty([[ i , direct * speed]])

if __name__ == '__main__':
	for i in range (300, 19):
		board.set_motor_duty([[1, -i], [2, i], [3, -i], [4,i]])
		time.sleep(0.3)
#	board.set_motor_duty([[1, -30], [2, -30], [3, -30], [4,-30]])
#	time.sleep(4)
	board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])
'''    while True:
        Forward(35)
        time.sleep(6)
        Shift(35,"left")
        time.sleep(3)
        Shift(35,"right")
        time.sleep(3)
        Forward(0)
        if not start:
            board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])  # Turn off all motors
            print('Closed')
            break
'''    
    
        
