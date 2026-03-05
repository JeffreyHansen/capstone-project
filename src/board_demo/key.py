#!/usr/bin/env python3
# encoding:utf-8
import time
import gpiod
import ros_robot_controller_sdk as rrc
import signal
print('''
**********************************************************
****Function: Hiwonder Raspberry Pi Expansion Board, Button Control RGB LED****
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

def handle_sigint(signal, frame):
    global start
    start = False
    board.set_rgb([[1, 0, 0, 0], [2, 0, 0, 0]])  # Set RGB to black (off)

# Register signal handler to catch Ctrl+C signal
signal.signal(signal.SIGINT, handle_sigint)

try:
    key1_pin = 13
    chip = gpiod.Chip("gpiochip4")
    key1 = chip.get_line(key1_pin)
    key1.request(consumer="key1", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

    key2_pin = 23
    key2 = chip.get_line(key2_pin)
    key2.request(consumer="key2", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

    while start:
        # Check button status
        key1_state = key1.get_value()
        key2_state = key2.get_value()

        if key1_state == 0:
            # Button 1 pressed, set RGB to red
            board.set_rgb([[1, 255, 0, 0], [2, 255, 0, 0]])

        if key2_state == 0:
            # Button 2 pressed, set RGB to blue
            board.set_rgb([[1, 0, 0, 255], [2, 0, 0, 255]])

        print('\rkey1: {} key2: {}'.format(key1_state, key2_state), end='', flush=True)  # Print key status
        time.sleep(0.001)

except Exception as e:
    print('Exception occurred:', str(e))
    print('Buttons are occupied by hw_button_scan by default, need to stop the service first')
    print('sudo systemctl stop hw_button_scan.service')

finally:
    if chip:
        chip.close()