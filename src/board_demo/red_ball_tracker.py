#!/usr/bin/env python3
# encoding: utf-8
"""
Red Ball Tracking with Camera Gimbal and Chassis Following
This script detects and tracks a red ball using:
- Camera gimbal (PWM servos) for visual tracking
- 4-wheel DC motors for chassis movement
- PID control for smooth tracking behavior
"""

import cv2
import time
import signal
import numpy as np
import ros_robot_controller_sdk as rrc


class PID:
    """Simple PID controller"""
    def __init__(self, P=0.0, I=0.0, D=0.0):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
        self.int_error = 0.0

    def update(self, feedback_value):
        """Calculate PID output based on feedback value"""
        error = self.SetPoint - feedback_value
        
        self.PTerm = self.Kp * error
        self.ITerm += error
        self.DTerm = error - self.last_error
        
        self.last_error = error
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
        
        return self.output

    def clear(self):
        """Reset PID controller"""
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0
        self.int_error = 0.0


class RedBallTracker:
    """Red ball tracking with camera gimbal and chassis control"""
    
    def __init__(self, board):
        self.board = board
        self.running = True
        
        # Camera settings
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        
        # Image processing size (smaller for faster processing)
        self.process_size = (320, 240)
        
        # Red color detection in HSV color space
        # Adjust these values if needed for your specific red ball
        self.lower_red1 = np.array([0, 100, 100])      # Lower red range
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])    # Upper red range
        self.upper_red2 = np.array([180, 255, 255])
        
        # Servo (camera gimbal) parameters
        self.servo_x = 1500  # Horizontal servo (servo 2)
        self.servo_y = 1500  # Vertical servo (servo 1)
        self.servo_min_x = 800
        self.servo_max_x = 2200
        self.servo_min_y = 1200
        self.servo_max_y = 1900
        
        # Servo PID controllers
        self.servo_x_pid = PID(P=0.3, I=0.05, D=0.01)
        self.servo_y_pid = PID(P=0.3, I=0.05, D=0.01)
        
        # Chassis (motor) parameters
        self.motor_max_speed = 60  # Maximum motor duty cycle
        self.motor_min_speed = 25  # Minimum motor duty cycle to overcome friction
        
        # Chassis PID controllers
        self.chassis_x_pid = PID(P=0.15, I=0.01, D=0.005)  # Left/right rotation
        self.chassis_y_pid = PID(P=0.10, I=0.005, D=0.003)  # Forward/backward
        
        # Tracking parameters
        self.min_ball_radius = 10   # Minimum radius to consider as ball
        self.target_ball_radius = 80  # Desired ball size (distance from robot)
        self.lost_target_threshold = 30  # Frames before considering target lost
        self.lost_target_count = 0
        
        # Deadzone thresholds (ignore small errors)
        self.servo_deadzone = 15
        self.chassis_deadzone = 20
        
        # Display window
        self.show_debug = True
        
        # Initialize servos to center position
        self.set_servo_position(self.servo_x, self.servo_y)
        time.sleep(0.5)
        
        print("Red Ball Tracker initialized!")
        print("- Camera resolution: 640x480")
        print("- Processing resolution: {}x{}".format(*self.process_size))
        print("- Press 'q' to quit")

    def set_servo_position(self, x, y):
        """Set camera gimbal servo positions"""
        # Servo 2 = X-axis (pan/horizontal)
        # Servo 1 = Y-axis (tilt/vertical)
        self.board.pwm_servo_set_position(0.02, [[2, int(x)], [1, int(y)]])

    def set_motor_speeds(self, speed_1, speed_2, speed_3, speed_4):
        """
        Set individual motor speeds
        Motor layout (viewed from top):
        [1]  [2]
         \\  //
         [CAR]
         //  \\
        [3]  [4]
        
        For forward: all motors same direction (alternating signs due to mounting)
        For rotation: left/right motors opposite directions
        """
        self.board.set_motor_duty([
            [1, int(speed_1)],
            [2, int(speed_2)],
            [3, int(speed_3)],
            [4, int(speed_4)]
        ])

    def move_robot(self, forward_speed, rotation_speed):
        """
        Move robot with forward and rotation speeds
        forward_speed: -100 to 100 (negative = backward)
        rotation_speed: -100 to 100 (negative = left, positive = right)
        """
        # Combine forward and rotation speeds for each motor
        # Motors 1,3 on one side, Motors 2,4 on other side
        left_speed = forward_speed - rotation_speed
        right_speed = forward_speed + rotation_speed
        
        # Clip speeds to valid range
        left_speed = np.clip(left_speed, -100, 100)
        right_speed = np.clip(right_speed, -100, 100)
        
        # Set motor speeds with appropriate sign for motor orientation
        self.set_motor_speeds(-left_speed, right_speed, -left_speed, right_speed)

    def stop_robot(self):
        """Stop all motors"""
        self.set_motor_speeds(0, 0, 0, 0)

    def detect_red_ball(self, frame):
        """
        Detect red ball in frame
        Returns: (x, y, radius) of detected ball, or None if not found
        """
        # Resize for faster processing
        small_frame = cv2.resize(frame, self.process_size)
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)
        
        # Create masks for red color (red wraps around HSV hue)
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Morphological operations to remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) == 0:
            return None, mask
        
        # Find largest contour (assumed to be the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Get minimum enclosing circle
        ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
        
        # Scale back to original frame size
        scale_x = frame.shape[1] / self.process_size[0]
        scale_y = frame.shape[0] / self.process_size[1]
        
        x = int(x * scale_x)
        y = int(y * scale_y)
        radius = int(radius * scale_x)
        
        # Only return if radius is above minimum threshold
        if radius < self.min_ball_radius:
            return None, mask
        
        return (x, y, radius), mask

    def update_servo_tracking(self, ball_x, ball_y, frame_width, frame_height):
        """Update servo positions to center ball in frame"""
        # Apply deadzone
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        if abs(ball_x - center_x) < self.servo_deadzone:
            ball_x = center_x
        if abs(ball_y - center_y) < self.servo_deadzone:
            ball_y = center_y
        
        # Update PID controllers
        self.servo_x_pid.SetPoint = center_x
        self.servo_x_pid.update(ball_x)
        servo_x_adjustment = int(self.servo_x_pid.output)
        
        self.servo_y_pid.SetPoint = center_y
        self.servo_y_pid.update(ball_y)
        servo_y_adjustment = int(self.servo_y_pid.output)
        
        # Apply adjustments
        self.servo_x += servo_x_adjustment
        self.servo_y -= servo_y_adjustment  # Inverted for servo orientation
        
        # Constrain to servo limits
        self.servo_x = int(np.clip(self.servo_x, self.servo_min_x, self.servo_max_x))
        self.servo_y = int(np.clip(self.servo_y, self.servo_min_y, self.servo_max_y))
        
        # Send to servos
        self.set_servo_position(self.servo_x, self.servo_y)

    def update_chassis_tracking(self, ball_x, radius, frame_width):
        """Update chassis motors to follow ball"""
        center_x = frame_width / 2
        
        # Calculate rotation needed (based on horizontal position)
        x_error = ball_x - center_x
        
        # Apply deadzone
        if abs(x_error) < self.chassis_deadzone:
            x_error = 0
        
        # Update rotation PID
        self.chassis_x_pid.SetPoint = 0
        rotation_speed = self.chassis_x_pid.update(x_error)
        
        # Calculate forward speed (based on ball size/distance)
        distance_error = self.target_ball_radius - radius
        
        # Update forward PID
        self.chassis_y_pid.SetPoint = 0
        forward_speed = self.chassis_y_pid.update(distance_error)
        
        # Apply minimum speed threshold
        if abs(forward_speed) > 0 and abs(forward_speed) < self.motor_min_speed:
            forward_speed = self.motor_min_speed if forward_speed > 0 else -self.motor_min_speed
        
        # Constrain speeds
        forward_speed = np.clip(forward_speed, -self.motor_max_speed, self.motor_max_speed)
        rotation_speed = np.clip(rotation_speed, -self.motor_max_speed, self.motor_max_speed)
        
        # Move robot
        self.move_robot(forward_speed, rotation_speed)

    def run(self):
        """Main tracking loop"""
        print("\nStarting red ball tracking...")
        print("Tracking parameters:")
        print(f"  - Minimum ball radius: {self.min_ball_radius} pixels")
        print(f"  - Target ball radius: {self.target_ball_radius} pixels")
        print(f"  - Servo deadzone: {self.servo_deadzone} pixels")
        print(f"  - Chassis deadzone: {self.chassis_deadzone} pixels")
        print()
        
        if self.show_debug:
            cv2.namedWindow("Red Ball Tracking", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Detection Mask", cv2.WINDOW_NORMAL)
        
        try:
            while self.running:
                # Capture frame
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to capture frame from camera")
                    break
                
                # Detect red ball
                detection, mask = self.detect_red_ball(frame)
                
                if detection is not None:
                    ball_x, ball_y, radius = detection
                    self.lost_target_count = 0
                    
                    # Update servo tracking (always track with camera)
                    self.update_servo_tracking(ball_x, ball_y, frame.shape[1], frame.shape[0])
                    
                    # Update chassis tracking (move robot toward ball)
                    self.update_chassis_tracking(ball_x, radius, frame.shape[1])
                    
                    # Draw detection on frame
                    if self.show_debug:
                        cv2.circle(frame, (ball_x, ball_y), radius, (0, 255, 0), 3)
                        cv2.circle(frame, (ball_x, ball_y), 5, (0, 0, 255), -1)
                        
                        # Display tracking info
                        info_text = [
                            f"Ball: ({ball_x}, {ball_y})",
                            f"Radius: {radius}px",
                            f"Servo X: {self.servo_x}",
                            f"Servo Y: {self.servo_y}",
                            f"Status: TRACKING"
                        ]
                        for i, text in enumerate(info_text):
                            cv2.putText(frame, text, (10, 30 + i*25), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                else:
                    # Target lost
                    self.lost_target_count += 1
                    
                    if self.lost_target_count > self.lost_target_threshold:
                        # Stop robot after losing target for too long
                        self.stop_robot()
                        
                        if self.show_debug:
                            cv2.putText(frame, "Status: TARGET LOST - STOPPED", (10, 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    else:
                        if self.show_debug:
                            cv2.putText(frame, f"Status: SEARCHING ({self.lost_target_count})", (10, 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                
                # Show debug windows
                if self.show_debug:
                    cv2.imshow("Red Ball Tracking", frame)
                    cv2.imshow("Detection Mask", mask)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        print("\nQuitting...")
                        break
                
                # Small delay to prevent CPU overload
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        print("Cleaning up...")
        
        # Stop motors
        self.stop_robot()
        
        # Center servos
        self.set_servo_position(1500, 1500)
        
        # Release camera
        self.camera.release()
        
        # Close windows
        if self.show_debug:
            cv2.destroyAllWindows()
        
        print("Cleanup complete")

    def handle_shutdown(self, signum, frame):
        """Handle shutdown signal"""
        print("\nShutdown signal received")
        self.running = False


def main():
    print('''
**********************************************************
****Red Ball Tracking with Camera Gimbal and Chassis****
**********************************************************
----------------------------------------------------------
Official website: https://www.hiwonder.com
----------------------------------------------------------
This script tracks a red ball using:
  - Camera gimbal (PWM servos 1 & 2)
  - 4-wheel DC motors for robot movement
  - PID control for smooth tracking

Controls:
  - Press 'q' in the video window to quit
  - Press Ctrl+C in terminal to stop
----------------------------------------------------------
''')
    
    # Initialize board
    print("Connecting to robot controller...")
    board = rrc.Board()
    time.sleep(0.5)
    
    # Create tracker
    tracker = RedBallTracker(board)
    
    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, tracker.handle_shutdown)
    signal.signal(signal.SIGTERM, tracker.handle_shutdown)
    
    # Run tracking
    tracker.run()


if __name__ == "__main__":
    main()
