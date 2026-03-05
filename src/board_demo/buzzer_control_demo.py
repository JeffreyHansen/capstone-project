import time
import ros_robot_controller_sdk as rrc

print('''
**********************************************************
****Function: Hiwonder Raspberry Pi Expansion Board, Buzzer Control****
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
board.set_buzzer(1900, 0.1, 0.9, 1) # At 1900Hz frequency, beep for 0.1s, off for 0.9s, repeat 1 time
time.sleep(2)
board.set_buzzer(1000, 0.5, 0.5, 0) # At 1000Hz frequency, beep for 0.5s, off for 0.5s, repeat continuously
time.sleep(3)
board.set_buzzer(1000, 0.0, 0.0, 1) # Turn off
