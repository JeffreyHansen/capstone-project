#!/usr/bin/python3
"""
Simple Ball Tracker - Pan Camera & Rotate Robot

Logic:
- Camera pans to center the ball in frame
- Pan servo position directly indicates ball direction relative to robot
- Robot rotates wheels until camera pan servo returns to center
- Once camera is forward-facing (pan centered), ball is straight ahead
- No forward/backward movement, no tilt control
"""

import cv2
import numpy as np
import ros_robot_controller_sdk as rrc
import time
from collections import deque

board = rrc.Board()
cap = cv2.VideoCapture(0)

# =============================================================================
# SERVO CONFIGURATION
# =============================================================================
PAN_ID = 2
TILT_ID = 1

PAN_CENTER = 1500
PAN_MIN = 1000
PAN_MAX = 2000
PAN_RANGE = PAN_MAX - PAN_MIN  # 1000 PWM units = 180°

TILT_CENTER = 1500
TILT_MIN = 1000
TILT_MAX = 2000

pan_pos = PAN_CENTER
tilt_pos = TILT_CENTER

def set_camera_pan(pan_pwm):
    """Set camera pan only, tilt is always centered"""
    global pan_pos
    pan_pwm = max(PAN_MIN, min(PAN_MAX, int(pan_pwm)))
    board.pwm_servo_set_position(30, [[PAN_ID, pan_pwm], [TILT_ID, TILT_CENTER]])
    pan_pos = pan_pwm

# =============================================================================
# MOTOR CONFIGURATION
# =============================================================================
speed_rotate = 20  # Motor duty cycle for rotation

def stop():
    """Stop all motors"""
    board.set_motor_duty([[1, 0], [2, 0], [3, 0], [4, 0]])

def rotate_left():
    """Rotate robot counter-clockwise (all motors same polarity)"""
    board.set_motor_duty([
        [1, speed_rotate],
        [2, speed_rotate],
        [3, speed_rotate],
        [4, speed_rotate]
    ])

def rotate_right():
    """Rotate robot clockwise (all motors reversed polarity)"""
    board.set_motor_duty([
        [1, -speed_rotate],
        [2, -speed_rotate],
        [3, -speed_rotate],
        [4, -speed_rotate]
    ])

# =============================================================================
# BALL DETECTION
# =============================================================================
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])

radius_history = deque(maxlen=8)

def smooth_radius(r):
    """Smooth radius with history"""
    radius_history.append(r)
    return sum(radius_history) / len(radius_history) if radius_history else r

def detect_ball(frame):
    """
    Detect red ball using HSV color range.
    Returns (x, y, radius) or None if not found.
    """
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2

    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if radius > 5:
            return int(x), int(y), int(radius)

    return None

# =============================================================================
# CAMERA PAN CONTROL
# =============================================================================
class CameraTracker:
    """
    Tracks ball in frame and adjusts camera pan to center it.
    Pan servo range maps to ~180° (-90° to +90°).
    """
    def __init__(self, frame_w=640, frame_h=480):
        self.frame_w = frame_w
        self.frame_h = frame_h
        self.pan_smooth = 0.0
        
        # Tracking parameters
        self.center_margin = 40  # pixels - don't pan if within this distance of center
        self.pan_gain = 1.2  # PWM units per pixel error
    
    def update(self, ball_detect):
        """
        Input: (x, y, radius) or None
        Output: (desired_pan_pwm, needs_pan)
        
        Pan to center the ball horizontally in frame.
        """
        if ball_detect is None:
            return PAN_CENTER, False
        
        x, y, radius = ball_detect
        frame_center_x = self.frame_w / 2
        
        # Calculate horizontal error
        x_error = x - frame_center_x
        
        # Only pan if error is significant
        if abs(x_error) < self.center_margin:
            # Ball roughly centered - minimize movement
            return PAN_CENTER, False
        
        # Calculate pan adjustment (smooth)
        pan_correction = x_error * self.pan_gain
        
        # Clamp to servo range
        desired_pan = PAN_CENTER + pan_correction
        desired_pan = max(PAN_MIN, min(PAN_MAX, int(desired_pan)))
        
        return desired_pan, True

camera_tracker = CameraTracker()

# =============================================================================
# ROBOT ROTATION CONTROL
# =============================================================================
class RotationController:
    """
    Uses pan servo position to determine rotation direction needed to
    align robot body with camera.
    
    Key insight:
    - If pan servo is at PAN_CENTER, ball is straight ahead
    - If pan servo is at PAN_MAX (right), ball is to the right
    - Robot should rotate right until pan servo comes back to PAN_CENTER
    """
    def __init__(self):
        self.pan_center_deadband = 30  # PWM units - margin where robot doesn't rotate
        self.last_command = None
    
    def get_rotation_command(self, pan_pwm):
        """
        Decides whether to rotate left, right, or stop based on pan position.
        
        Returns: ("left", "right", or "stop")
        """
        offset_from_center = pan_pwm - PAN_CENTER
        
        # Within deadband - stop rotation
        if abs(offset_from_center) < self.pan_center_deadband:
            return "stop"
        
        # Pan is to the right (positive offset) - rotate right
        if offset_from_center > 0:
            return "right"
        
        # Pan is to the left (negative offset) - rotate left
        return "left"

rotation_controller = RotationController()

# =============================================================================
# INITIALIZATION
# =============================================================================
set_camera_pan(PAN_CENTER)
print("=" * 70)
print("SIMPLE BALL TRACKER - Camera Pan + Robot Rotation")
print("=" * 70)
print("Logic: Camera pans to center ball, robot rotates until camera faces forward")
print("Controls: Q to quit")
print("=" * 70)

last_frame_time = time.time()
frame_count = 0
search_pan = PAN_CENTER
search_direction = 1
search_start_time = None
SEARCH_PAN_STEP = 150
SEARCH_UPDATE_DELAY = 0.015

# =============================================================================
# MAIN LOOP
# =============================================================================
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()
        dt = current_time - last_frame_time
        last_frame_time = current_time
        frame_count += 1

        h, w = frame.shape[:2]
        
        # Detect ball
        ball_detect = detect_ball(frame)
        
        if ball_detect is not None:
            x, y, radius = ball_detect
            radius = smooth_radius(radius)
            
            # Draw detection
            cv2.circle(frame, (x, y), int(radius), (0, 0, 255), 3)
            
            # Reset search timer
            search_start_time = None
            
            # ===== CAMERA CONTROL =====
            desired_pan, needs_pan = camera_tracker.update(ball_detect)
            set_camera_pan(desired_pan)
            
            # ===== ROBOT ROTATION CONTROL =====
            rotation_cmd = rotation_controller.get_rotation_command(pan_pos)
            
            if rotation_cmd == "left":
                rotate_left()
            elif rotation_cmd == "right":
                rotate_right()
            else:
                stop()
            
            # Debug output
            if frame_count % 15 == 0:
                pan_offset = pan_pos - PAN_CENTER
                print(f"Ball: ({x}, {y}) r={radius:.0f} | Pan offset: {pan_offset:+4.0f} PWM | Rotate: {rotation_cmd}")
        
        else:
            # ===== SEARCH MODE (no ball detected) =====
            # Sweep camera pan left-right to find ball
            if search_start_time is None:
                search_start_time = current_time
                search_pan = PAN_CENTER
                search_direction = 1
            
            if current_time - search_start_time > 0.0:  # Always searching
                search_pan += SEARCH_PAN_STEP * search_direction
                
                # Bounce at limits
                if search_pan >= PAN_MAX:
                    search_pan = PAN_MAX
                    search_direction = -1
                elif search_pan <= PAN_MIN:
                    search_pan = PAN_MIN
                    search_direction = 1
                
                set_camera_pan(search_pan)
                stop()  # Don't rotate body during search
            
            cv2.putText(frame, "SEARCHING...", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 2)
        
        # Display frame
        cv2.imshow("Ball Tracker - Simple", frame)
        
        if cv2.waitKey(1) == ord('q'):
            break

finally:
    # =============================================================================
    # CLEANUP
    # =============================================================================
    stop()
    set_camera_pan(PAN_CENTER)
    cap.release()
    cv2.destroyAllWindows()
    print("Tracker stopped.")
