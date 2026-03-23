#!/usr/bin/python3
import argparse
import cv2
import numpy as np
import ros_robot_controller_sdk as rrc
import time
from collections import deque

board = rrc.Board()
cap = cv2.VideoCapture(0)

# -----------------------------
# PWM SERVO SETTINGS
# -----------------------------
PAN_ID = 2
TILT_ID = 1

PAN_CENTER = 1500
PAN_RANGE_90_DEG = 500  # 90° left/right from center
PAN_MIN = PAN_CENTER - PAN_RANGE_90_DEG
PAN_MAX = PAN_CENTER + PAN_RANGE_90_DEG

TILT_CENTER = 1500
TILT_MIN = 1000
TILT_MAX = 2000

pan_pos = PAN_CENTER
tilt_pos = TILT_CENTER

PAN_FAST_MOVE_MS = 20
PAN_NORMAL_MOVE_MS = 35
PAN_LOCKOUT_STEP_PWM = 20
PAN_MOTION_SETTLE_SEC = 0.08

last_pan_command = PAN_CENTER
pan_motion_until = 0.0

def clamp_pan(pan):
    return max(PAN_MIN, min(PAN_MAX, int(pan)))

def move_servos(pan, tilt, move_time_ms=PAN_NORMAL_MOVE_MS):
    global last_pan_command, pan_motion_until
    pan = clamp_pan(pan)
    tilt = TILT_CENTER  # Vertical movement disabled
    board.pwm_servo_set_position(move_time_ms, [[PAN_ID, pan], [TILT_ID, tilt]])

    if abs(pan - last_pan_command) >= PAN_LOCKOUT_STEP_PWM:
        pan_motion_until = max(
            pan_motion_until,
            time.time() + (move_time_ms / 1000.0) + PAN_MOTION_SETTLE_SEC
        )

    last_pan_command = pan

def is_pan_in_motion():
    return time.time() < pan_motion_until

# -----------------------------
# ROBOT ORIENTATION TRACKING
# (estimates heading from motor commands)
# -----------------------------
class RobotOrientationTracker:
    def __init__(self):
        self.heading = 0.0  # degrees, 0 = forward
        self.last_motor_state = [0, 0, 0, 0]  # motors 1-4 duty cycle
        self.heading_history = deque(maxlen=10)
    
    def update_from_motors(self, motor_states, dt):
        """
        motor_states: [motor1_duty, motor2_duty, motor3_duty, motor4_duty]
        dt: time since last update (seconds)
        
        TurboPi omnidirectional drive:
        Motor 1 & 3 are diagonal pair (e.g., front-left, back-right)
        Motor 2 & 4 are diagonal pair (e.g., front-right, back-left)
        
        Rotation happens when opposing diagonals have opposite signs:
        rotate_right: motors 1,2 positive (or 3,4 negative)
        rotate_left: motors 1,2 negative (or 3,4 positive)
        """
        self.last_motor_state = motor_states
        
        # Estimate rotation from motor commands
        m1, m2, m3, m4 = motor_states
        
        # Rotation speed estimate based on this motor mapping:
        # rotate_left/right commands set all motors to same sign;
        # forward/backward commands approximately cancel to zero.
        rotation_bias = (m1 + m2 + m3 + m4) / 4.0
        
        # Rotation per frame (degrees)
        # Adjust ROTATION_GAIN based on your motor response
        ROTATION_GAIN = 1.0  # degrees per unit duty difference
        heading_delta = rotation_bias * ROTATION_GAIN * dt
        
        self.heading += heading_delta
        self.heading = self.heading % 360  # wrap to 0-360
        self.heading_history.append(self.heading)
    
    def get_heading(self):
        """Returns current estimated heading in degrees"""
        return self.heading
    
    def get_smooth_heading(self):
        """Returns smoothed heading"""
        if self.heading_history:
            return np.mean(self.heading_history)
        return self.heading

orientation_tracker = RobotOrientationTracker()

# -----------------------------
# GIMBAL STABILIZATION
# (keeps camera looking at fixed world direction when robot rotates)
# -----------------------------
class GimbalStabilizer:
    def __init__(self):
        self.target_world_heading = 0.0  # desired camera direction in world frame
        self.pan_offset_from_rotation = 0.0  # compensation for robot rotation
    
    def update_stabilization(self, robot_heading):
        """
        Calculates how much to offset pan servo to maintain world-fixed heading
        when robot rotates.
        
        If robot rotates 30° right, pan servo should rotate 30° left to stay fixed.
        """
        # Pan offset = -robot_heading (opposite rotation)
        self.pan_offset_from_rotation = -robot_heading
    
    def get_stabilized_pan(self, base_pan_pos, robot_heading):
        """
        Returns pan position adjusted for robot stabilization
        base_pan_pos: pan position before stabilization (from ball tracking)
        robot_heading: current robot heading
        """
        stabilization_pwm = robot_heading * 10 / 3.6  # convert degrees to PWM delta
        stabilized_pan = base_pan_pos + stabilization_pwm
        return stabilized_pan

gimbal_stabilizer = GimbalStabilizer()

# -----------------------------
# BALL TRACKING WITH VELOCITY
# (only moves gimbal when ball exits frame)
# -----------------------------
class BallVelocityTracker:
    def __init__(self, frame_w=640, frame_h=480):
        self.frame_w = frame_w
        self.frame_h = frame_h
        
        self.position_history = deque(maxlen=5)
        self.velocity = (0.0, 0.0)
        
        # Gimbal only moves if ball goes beyond these margins
        self.x_margin = 80      # pixels from left/right edge
        self.y_margin = 60      # pixels from top/bottom edge
        self.margin_vel_factor = 1.5  # predict further if ball moving fast
        
        self.last_pan_correction = 0
    
    def update(self, ball_detect, dt):
        """
        ball_detect: (x, y, radius) or None
        dt: time since last call
        returns: (pan_correction, tilt_correction, needs_adjustment)
        """
        if ball_detect is None:
            return 0, 0, False  # Camera stays still if ball not detected
        
        x, y, radius = ball_detect
        current_time = time.time()
        
        # Track position history
        self.position_history.append((x, y, current_time))
        
        # Calculate velocity (pixels/sec)
        if len(self.position_history) >= 2:
            x1, y1, t1 = self.position_history[0]
            x2, y2, t2 = self.position_history[-1]
            dt_vel = max(0.001, t2 - t1)
            
            self.velocity = (
                (x2 - x1) / dt_vel,
                (y2 - y1) / dt_vel
            )
        
        vx, vy = self.velocity
        
        # Predict where ball will be (220ms into future)
        predict_time = 0.22
        future_x = x + vx * predict_time
        future_y = y + vy * predict_time
        
        # Adaptive margin based on velocity
        vel_magnitude = np.sqrt(vx**2 + vy**2)
        dynamic_margin_x = self.x_margin + vel_magnitude * self.margin_vel_factor
        dynamic_margin_y = self.y_margin + vel_magnitude * self.margin_vel_factor
        
        needs_adjustment = False
        pan_correction = 0
        
        # ===== PAN (HORIZONTAL) =====
        # Ball moving off left edge?
        if future_x < dynamic_margin_x and vx < 0:
            # Snap pan right so ball re-enters from right side
            # How many pixels off-frame? Compute servo movement to recenter
            pixels_off = dynamic_margin_x - future_x
            pan_correction = pixels_off * 1.8  # aggressive snap correction
            needs_adjustment = True
        
        # Ball moving off right edge?
        elif future_x > self.frame_w - dynamic_margin_x and vx > 0:
            # Snap pan left
            pixels_off = future_x - (self.frame_w - dynamic_margin_x)
            pan_correction = -pixels_off * 1.8
            needs_adjustment = True
        
        else:
            # Ball in safe zone - small correction to keep centered
            center_x = self.frame_w / 2
            x_error = x - center_x
            # Only correct if significantly off-center
            if abs(x_error) > 20:
                pan_correction = x_error * 0.08
                needs_adjustment = True

        return int(pan_correction), 0, needs_adjustment

ball_tracker = BallVelocityTracker()

# -----------------------------
# TRACKING SETTINGS
# -----------------------------
CENTER_X_DEADBAND = 40
CENTER_Y_DEADBAND = 40

PAN_ROBOT_THRESHOLD = 250   # when robot should rotate

TARGET_RADIUS = 50
DISTANCE_DEADBAND = 8

# -----------------------------
# SEARCH SETTINGS
# -----------------------------
search_direction = 1
SEARCH_PAN_STEP = 220

last_seen_direction = 1
lost_start_time = None
SEARCH_DELAY = 0.01
last_search_time = 0

GIMBAL_TEST_SWITCH_SEC = 1.8
GIMBAL_TEST_ROTATE_SPEED = 40

parser = argparse.ArgumentParser()
parser.add_argument(
    "--gimbal-test",
    action="store_true",
    help="Rotate robot left/right repeatedly while stabilization keeps camera centered"
)
args = parser.parse_args()

# -----------------------------
# MOTOR SETTINGS
# -----------------------------
speed_forward = 25
speed_rotate = 12

# Store current motor state for orientation tracking
current_motor_state = [0, 0, 0, 0]

# -----------------------------
# COLOR DETECTION
# -----------------------------
lower_red1 = np.array([0,120,70])
upper_red1 = np.array([10,255,255])

lower_red2 = np.array([170,120,70])
upper_red2 = np.array([180,255,255])

radius_history = []
HISTORY_SIZE = 8

# -----------------------------
# MOTOR FUNCTIONS
# (updated to track motor state)
# -----------------------------
def stop():
    global current_motor_state
    current_motor_state = [0, 0, 0, 0]
    board.set_motor_duty([[1,0],[2,0],[3,0],[4,0]])

def forward():
    global current_motor_state
    current_motor_state = [-speed_forward, speed_forward, -speed_forward, speed_forward]
    board.set_motor_duty([[1,-speed_forward],[2,speed_forward],[3,-speed_forward],[4,speed_forward]])

def backward():
    global current_motor_state
    current_motor_state = [speed_forward, -speed_forward, speed_forward, -speed_forward]
    board.set_motor_duty([[1,speed_forward],[2,-speed_forward],[3,speed_forward],[4,-speed_forward]])

def rotate_left(speed=None):
    global current_motor_state
    speed_cmd = speed_rotate if speed is None else speed
    current_motor_state = [speed_cmd, speed_cmd, speed_cmd, speed_cmd]
    board.set_motor_duty([[1,speed_cmd],[2,speed_cmd],[3,speed_cmd],[4,speed_cmd]])

def rotate_right(speed=None):
    global current_motor_state
    speed_cmd = speed_rotate if speed is None else speed
    current_motor_state = [-speed_cmd, -speed_cmd, -speed_cmd, -speed_cmd]
    board.set_motor_duty([[1,-speed_cmd],[2,-speed_cmd],[3,-speed_cmd],[4,-speed_cmd]])

# Omnidirectional holonomic movement
def move_omni(vx, vy, vrot):
    """
    OmniDirection drive for 4-wheel TurboPi
    vx: forward/backward velocity
    vy: left/right velocity  
    vrot: rotation velocity
    """
    global current_motor_state
    # Simplified holonomic kinematics (adjust for your actual motor layout)
    m1 = -vx + vy + vrot
    m2 = vx + vy - vrot
    m3 = -vx - vy - vrot
    m4 = vx - vy + vrot
    
    current_motor_state = [m1, m2, m3, m4]
    board.set_motor_duty([[1,m1],[2,m2],[3,m3],[4,m4]])

# -----------------------------
# HELPERS
# -----------------------------
def smooth_radius(r):
    radius_history.append(r)
    if len(radius_history) > HISTORY_SIZE:
        radius_history.pop(0)
    return sum(radius_history)/len(radius_history)

def detect_ball(frame):
    blurred = cv2.GaussianBlur(frame,(11,11),0)
    hsv = cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv,lower_red1,upper_red1)
    mask2 = cv2.inRange(hsv,lower_red2,upper_red2)
    mask = mask1 + mask2

    mask = cv2.erode(mask,None,iterations=2)
    mask = cv2.dilate(mask,None,iterations=2)

    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours,key=cv2.contourArea)
        ((x,y),radius) = cv2.minEnclosingCircle(c)
        if radius > 5:
            return int(x),int(y),int(radius)

    return None

# -----------------------------
# INIT
# -----------------------------
move_servos(pan_pos, tilt_pos)
print("TurboPi Soccer Bot - Gimbal Stabilized Version")
print("Features: World-fixed gimbal + Smart ball tracking + Velocity prediction")
if args.gimbal_test:
    print("Gimbal test mode active: robot oscillates while camera stays centered")

last_frame_time = time.time()
frame_count = 0
test_direction = 1
last_test_switch_time = time.time()

# -----------------------------
# MAIN LOOP
# -----------------------------
while True:

    ret, frame = cap.read()
    if not ret:
        break

    current_time = time.time()
    dt = current_time - last_frame_time
    last_frame_time = current_time
    frame_count += 1

    if is_pan_in_motion() and not args.gimbal_test:
        result = None
    else:
        result = detect_ball(frame)

    h, w = frame.shape[:2]
    cx = w/2
    cy = h/2

    # Update orientation tracker
    orientation_tracker.update_from_motors(current_motor_state, dt)
    robot_heading = orientation_tracker.get_smooth_heading()
    
    # Update gimbal stabilization
    gimbal_stabilizer.update_stabilization(robot_heading)

    if args.gimbal_test:
        if current_time - last_test_switch_time >= GIMBAL_TEST_SWITCH_SEC:
            test_direction *= -1
            last_test_switch_time = current_time

        if test_direction > 0:
            rotate_left(GIMBAL_TEST_ROTATE_SPEED)
        else:
            rotate_right(GIMBAL_TEST_ROTATE_SPEED)

        pan_stabilized = gimbal_stabilizer.get_stabilized_pan(PAN_CENTER, robot_heading)
        pan_stabilized = clamp_pan(pan_stabilized)
        move_servos(pan_stabilized, TILT_CENTER, move_time_ms=PAN_FAST_MOVE_MS)
        pan_pos = pan_stabilized

        cv2.putText(frame, "GIMBAL TEST MODE", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

    elif result is not None:

        x, y, radius = result
        radius = smooth_radius(radius)

        cv2.circle(frame, (x, y), int(radius), (0, 0, 255), 3)

        x_error = x - cx
        last_seen_direction = 1 if x_error > 0 else -1
        lost_start_time = None

        # ===== GIMBAL CONTROL =====
        # Get ball-tracking gimbal adjustments
        pan_ball_correction, tilt_ball_correction, needs_adjustment = ball_tracker.update(
            result, dt
        )
        
        # Apply stabilization (compensate for robot rotation)
        pan_stabilized = PAN_CENTER + pan_ball_correction
        pan_stabilized = gimbal_stabilizer.get_stabilized_pan(pan_stabilized, robot_heading)
        pan_stabilized = clamp_pan(pan_stabilized)
        tilt_stabilized = TILT_CENTER
        
        pan_delta = abs(pan_stabilized - pan_pos)
        move_time = PAN_FAST_MOVE_MS if pan_delta >= PAN_LOCKOUT_STEP_PWM else PAN_NORMAL_MOVE_MS
        move_servos(pan_stabilized, tilt_stabilized, move_time_ms=move_time)
        pan_pos = pan_stabilized
        tilt_pos = TILT_CENTER

        # ===== ROBOT MOVEMENT LOGIC =====
        # The gimbal is world-fixed now, so pan_pos tells us ball's absolute position
        
        if pan_pos < PAN_CENTER - PAN_ROBOT_THRESHOLD:
            # Ball is on left side of field - rotate left
            rotate_left()

        elif pan_pos > PAN_CENTER + PAN_ROBOT_THRESHOLD:
            # Ball is on right side - rotate right
            rotate_right()

        else:
            # Ball roughly ahead - approach with forward/backward
            radius_error = radius - TARGET_RADIUS

            if abs(radius_error) < DISTANCE_DEADBAND:
                stop()
            elif radius_error > 0:
                backward()
            else:
                forward()
        
        # Debug info
        vx, vy = ball_tracker.velocity
        if frame_count % 10 == 0:  # Print every 10 frames
            print(f"Robot heading: {robot_heading:.1f}° | Ball vel: ({vx:.0f}, {vy:.0f}) px/s | Pan: {pan_pos:.0f}")

    else:
        # ===== SEARCH MODE (no ball detected): quick pan sweep only =====
        current_time = time.time()

        if lost_start_time is None:
            lost_start_time = current_time
            search_direction = last_seen_direction

        if current_time - last_search_time > SEARCH_DELAY:
            last_search_time = current_time

            pan_pos += SEARCH_PAN_STEP * search_direction

            if pan_pos >= PAN_MAX:
                pan_pos = PAN_MAX
                search_direction = -1
            elif pan_pos <= PAN_MIN:
                pan_pos = PAN_MIN
                search_direction = 1

            # Apply stabilization even during search
            pan_stabilized = gimbal_stabilizer.get_stabilized_pan(pan_pos, robot_heading)
            pan_stabilized = clamp_pan(pan_stabilized)
            move_servos(pan_stabilized, TILT_CENTER, move_time_ms=PAN_FAST_MOVE_MS)

            # Do not rotate robot during camera search; keep chassis still
            stop()

    cv2.imshow("Ball Tracker - Gimbal Stabilized", frame)

    if cv2.waitKey(1) == ord('q'):
        break

# ===== CLEANUP =====
stop()
board.pwm_servo_set_position(200, [[PAN_ID, PAN_CENTER], [TILT_ID, TILT_CENTER]])
cap.release()
cv2.destroyAllWindows()
