import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc

print('''
**********************************************************
****Function: Hiwonder Raspberry Pi Expansion Board, PWM Servo Speed Control****
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
        board.pwm_servo_set_position(0.5, [[1, 1500]]) # Set servo 1 pulse width to 1500
        time.sleep(0.5)
        board.pwm_servo_set_position(0.5, [[1, 1000]]) # Set servo 1 pulse width to 1000
        time.sleep(0.5)
        board.pwm_servo_set_position(0.5, [[1, 500]]) # Set servo 1 pulse width to 500
        time.sleep(0.5)
        board.pwm_servo_set_position(1, [[1, 1000]]) # Set servo 1 pulse width to 1000
        time.sleep(1)
        board.pwm_servo_set_position(1, [[1, 1500]]) # Set servo 1 pulse width to 1500
        time.sleep(1)
        board.pwm_servo_set_position(1, [[1, 2000]]) # Set servo 1 pulse width to 2000
        time.sleep(1)
        board.pwm_servo_set_position(1, [[1, 2500]]) # Set servo 1 pulse width to 2500
        time.sleep(1)
        if not start:
            board.pwm_servo_set_position(1, [[1, 1500]]) # Set servo 1 pulse width to 1500
            time.sleep(1)
            print('Closed')
            break