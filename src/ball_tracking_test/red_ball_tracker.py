#!/usr/bin/env python3
# encoding: utf-8
"""
Neon Green Ball Tracking with Camera Gimbal and Chassis Following
This script detects and tracks a neon green ball using:
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
    
    def __init__(self, board, camera_index=0):
        self.board = board
        self.running = True
        
        # Camera settings - try multiple devices if first one fails
        self.camera = None
        camera_devices = [camera_index, 1, 2]  # Try video0, video1, video2
        
        for device in camera_devices:
            print(f"Attempting to open camera /dev/video{device}...")
            test_camera = cv2.VideoCapture(device)
            if test_camera.isOpened():
                print(f"✓ Successfully opened /dev/video{device}")
                self.camera = test_camera
                break
            else:
                print(f"✗ Failed to open /dev/video{device}")
                test_camera.release()
        
        if self.camera is None or not self.camera.isOpened():
            print("\n" + "="*60)
            print("ERROR: Cannot open any camera device!")
            print("="*60)
            print("\nPossible causes:")
            print("1. Camera is in use by another process (Docker/ROS2)")
            print("2. Camera is not connected")
            print("3. Permission issues")
            print("\nTo diagnose, run:")
            print("  ./check_camera.sh")
            print("\nOr manually check:")
            print("  sudo lsof /dev/video*")
            print("  docker ps")
            print("\nTo stop Docker containers:")
            print("  docker stop <container_name>")
            print("="*60)
            raise RuntimeError("Failed to initialize camera")
        
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        
        # Try to reduce motion blur by increasing shutter speed (lower exposure)
        # Note: Not all cameras support these settings
        self.camera.set(cv2.CAP_PROP_EXPOSURE, -6)  # Faster shutter, less blur
        self.camera.set(cv2.CAP_PROP_GAIN, 2)  # Compensate with gain
        
        # Image processing size (smaller for faster processing)
        self.process_size = (320, 240)
        
        # Neon green color detection in HSV color space
        # Adjust these values if needed for your specific neon green ball
        # Neon green typically has hue 60-90, high saturation, high value
        self.lower_green = np.array([40, 100, 100])    # Green range
        self.upper_green = np.array([90, 255, 255])
        
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
        
        # Chassis (motor) parameters - REDUCED to prevent blur
        self.motor_max_speed = 35  # Reduced from 60 to prevent motion blur
        self.motor_min_speed = 15  # Reduced minimum threshold
        
        # Chassis PID controllers - LESS AGGRESSIVE to prevent oscillation
        self.chassis_x_pid = PID(P=0.06, I=0.001, D=0.008)  # Strafe left/right
        self.chassis_rot_pid = PID(P=0.08, I=0.002, D=0.01)  # Rotation (yaw)
        self.chassis_y_pid = PID(P=0.05, I=0.001, D=0.005)  # Forward/backward
        
        # Detection smoothing - prevent oscillation from noisy detections
        self.detection_history = []
        self.history_size = 5  # Average over last 5 detections
        self.smoothed_ball_x = None
        self.smoothed_ball_y = None
        self.smoothed_radius = None
        
        # Deadzone thresholds - INCREASED to prevent jitter
        self.servo_deadzone = 25  # Increased from 15
        self.chassis_deadzone = 35  # Increased from 20
        self.rotation_deadzone = 30  # Separate deadzone for rotation
        self.min_ball_radius = 10   # Minimum radius to consider as ball
        self.target_ball_radius = 80  # Desired ball size (distance from robot)
        self.lost_target_threshold = 30  # Frames before considering target lost
        self.lost_target_count = 0
        
        # Display window
        self.show_debug = True
        
        # Initialize servos to center position
        self.set_servo_position(self.servo_x, self.servo_y)
        time.sleep(0.5)
        
        print("Neon Green Ball Tracker initialized!")
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
        Omnidirectional robot - Mecanum wheel layout (viewed from top):
        [1]  [2]     Motor numbering
         \\  //
         [CAR]
         //  \\
        [3]  [4]
        
        Wheel orientation (/ = forward-left, \\ = forward-right):
        \\  /   <- Motor 1 (back-left),  Motor 2 (back-right)
        [CAR]
        /  \\   <- Motor 3 (front-left), Motor 4 (front-right)
        """
        self.board.set_motor_duty([
            [1, int(speed_1)],
            [2, int(speed_2)],
            [3, int(speed_3)],
            [4, int(speed_4)]
        ])

    def move_robot_omni(self, forward_speed, strafe_speed, rotation_speed):
        """
        Move omnidirectional robot (mecanum wheels)
        
        Parameters:
        - forward_speed: -100 to 100 (negative = backward, positive = forward)
        - strafe_speed: -100 to 100 (negative = left, positive = right)
        - rotation_speed: -100 to 100 (negative = CCW, positive = CW)
        
        Mecanum wheel control equations:
        For typical mecanum configuration:
        - Motor 1 (front-left):  forward + strafe + rotation
        - Motor 2 (front-right): forward - strafe - rotation
        - Motor 3 (back-left):   forward - strafe + rotation
        - Motor 4 (back-right):  forward + strafe - rotation
        """
        # Calculate individual motor speeds
        m1 = forward_speed + strafe_speed + rotation_speed
        m2 = forward_speed - strafe_speed - rotation_speed
        m3 = forward_speed - strafe_speed + rotation_speed
        m4 = forward_speed + strafe_speed - rotation_speed
        
        # Normalize if any speed exceeds max
        max_speed = max(abs(m1), abs(m2), abs(m3), abs(m4))
        if max_speed > 100:
            scale = 100.0 / max_speed
            m1 *= scale
            m2 *= scale
            m3 *= scale
            m4 *= scale
        
        # Apply motor orientation (adjust signs based on your robot's wiring)
        # You may need to invert some of these - test and adjust!
        self.set_motor_speeds(-m1, m2, -m3, m4)

    def stop_robot(self):
        """Stop all motors"""
        self.set_motor_speeds(0, 0, 0, 0)

    def detect_red_ball(self, frame):
        """
        Detect neon green ball in frame
        Returns: (x, y, radius) of detected ball, or None if not found
        """
        # Resize for faster processing
        small_frame = cv2.resize(frame, self.process_size)
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)
        
        # Create mask for neon green color
        mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        
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
        self.servo_x = np.clip(self.servo_x, self.servo_min_x, self.servo_max_x)
        self.servo_y = np.clip(self.servo_y, self.servo_min_y, self.servo_max_y)
        
        # Apply servo positions
        self.set_servo_position(self.servo_x, self.servo_y)

    def smooth_detection(self, ball_x, ball_y, radius):
        """
        Smooth detection using moving average to prevent oscillation
        Returns smoothed values
        """
        # Add current detection to history
        self.detection_history.append((ball_x, ball_y, radius))
        
        # Keep only recent detections
        if len(self.detection_history) > self.history_size:
            self.detection_history.pop(0)
        
        # Calculate average
        if len(self.detection_history) > 0:
            avg_x = sum(d[0] for d in self.detection_history) / len(self.detection_history)
            avg_y = sum(d[1] for d in self.detection_history) / len(self.detection_history)
            avg_r = sum(d[2] for d in self.detection_history) / len(self.detection_history)
            return avg_x, avg_y, avg_r
        
        return ball_x, ball_y, radius

    def update_chassis_tracking(self, ball_x, ball_y, radius, frame_width, frame_height):
        """Update chassis motors to follow ball - OMNIDIRECTIONAL VERSION"""
        center_x = frame_width / 2
        center_y = frame_height / 2
        
        # Calculate errors
        x_error = ball_x - center_x  # Horizontal offset
        y_error = ball_y - center_y  # Vertical offset (can use for strategy)
        distance_error = radius - self.target_ball_radius  # Distance to ball
        
        # Apply deadzones to prevent jitter
        if abs(x_error) < self.rotation_deadzone:
            x_error = 0
        if abs(distance_error) < 10:  # Small deadzone for distance
            distance_error = 0
        
        # Strategy: Use rotation OR strafe, not both (prevents confusion)
        # If ball is far off-center, rotate. If close to center, strafe.
        if abs(x_error) > 100:
            # Ball is far off-center: ROTATE to face it
            self.chassis_rot_pid.SetPoint = 0
            rotation_speed = -self.chassis_rot_pid.update(x_error)  # INVERTED: positive x_error (right) -> rotate right (positive)
            strafe_speed = 0  # Don't strafe while rotating
        else:
            # Ball is near center: STRAFE to center it perfectly
            self.chassis_x_pid.SetPoint = 0
            strafe_speed = -self.chassis_x_pid.update(x_error)  # INVERTED: positive x_error (right) -> strafe right (positive)
            rotation_speed = 0  # Don't rotate while strafing
        
        # Forward/backward based on distance
        self.chassis_y_pid.SetPoint = 0
        forward_speed = self.chassis_y_pid.update(distance_error)  # Move forward when ball is far (small radius)
        
        # Apply minimum speed thresholds
        if 0 < abs(forward_speed) < self.motor_min_speed:
            forward_speed = self.motor_min_speed if forward_speed > 0 else -self.motor_min_speed
        if 0 < abs(strafe_speed) < self.motor_min_speed:
            strafe_speed = self.motor_min_speed if strafe_speed > 0 else -self.motor_min_speed
        if 0 < abs(rotation_speed) < self.motor_min_speed:
            rotation_speed = self.motor_min_speed if rotation_speed > 0 else -self.motor_min_speed
        
        # Constrain speeds to max
        forward_speed = np.clip(forward_speed, -self.motor_max_speed, self.motor_max_speed)
        strafe_speed = np.clip(strafe_speed, -self.motor_max_speed, self.motor_max_speed)
        rotation_speed = np.clip(rotation_speed, -self.motor_max_speed, self.motor_max_speed)
        
        # Move robot using omnidirectional control
        self.move_robot_omni(forward_speed, strafe_speed, rotation_speed)

    def run(self):
        """Main tracking loop"""
        print("Starting neon green ball tracking...")
        print(f"Target ball radius: {self.target_ball_radius} pixels")
        print(f"  - Servo deadzone: {self.servo_deadzone} pixels")
        print(f"  - Chassis deadzone: {self.chassis_deadzone} pixels")
        print(f"  - Rotation deadzone: {self.rotation_deadzone} pixels")
        print()
        
        if self.show_debug:
            cv2.namedWindow("Neon Green Ball Tracking", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Detection Mask", cv2.WINDOW_NORMAL)
        
        try:
            while self.running:
                # Capture frame
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to capture frame from camera")
                    break
                
                # Detect neon green ball
                detection, mask = self.detect_red_ball(frame)
                
                if detection is not None:
                    ball_x, ball_y, radius = detection
                    self.lost_target_count = 0
                    
                    # Smooth detection to prevent oscillation
                    smooth_x, smooth_y, smooth_r = self.smooth_detection(ball_x, ball_y, radius)
                    
                    # Update servo tracking (always track with camera)
                    self.update_servo_tracking(smooth_x, smooth_y, frame.shape[1], frame.shape[0])
                    
                    # Update chassis tracking (move robot toward ball)
                    self.update_chassis_tracking(smooth_x, smooth_y, smooth_r, frame.shape[1], frame.shape[0])
                    
                    # Draw detection on frame
                    if self.show_debug:
                        # Draw raw detection (red)
                        cv2.circle(frame, (ball_x, ball_y), radius, (0, 0, 255), 2)
                        # Draw smoothed detection (green)
                        cv2.circle(frame, (int(smooth_x), int(smooth_y)), int(smooth_r), (0, 255, 0), 3)
                        cv2.circle(frame, (int(smooth_x), int(smooth_y)), 5, (0, 255, 0), -1)
                        
                        # Display tracking info
                        info_text = [
                            f"Raw: ({ball_x}, {ball_y}) r={radius}",
                            f"Smooth: ({int(smooth_x)}, {int(smooth_y)}) r={int(smooth_r)}",
                            f"Servo X: {self.servo_x}, Y: {self.servo_y}",
                            f"History: {len(self.detection_history)} frames",
                            f"Status: TRACKING"
                        ]
                        for i, text in enumerate(info_text):
                            cv2.putText(frame, text, (10, 30 + i*25), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                else:
                    # Target lost
                    self.lost_target_count += 1
                    
                    # Clear detection history when target is lost
                    if self.lost_target_count > 3:
                        self.detection_history.clear()
                    
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
                    cv2.imshow("Neon Green Ball Tracking", frame)
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
    import sys
    
    print('''
**********************************************************
****Neon Green Ball Tracking - Gimbal and Chassis****
**********************************************************
----------------------------------------------------------
Official website: https://www.hiwonder.com
----------------------------------------------------------
This script tracks a neon green ball using:
  - Camera gimbal (PWM servos 1 & 2)
  - 4-wheel DC motors for robot movement
  - PID control for smooth tracking

Controls:
  - Press 'q' in the video window to quit
  - Press Ctrl+C in terminal to stop
----------------------------------------------------------
''')
    
    # Check for camera index argument
    camera_index = 0
    if len(sys.argv) > 1:
        try:
            camera_index = int(sys.argv[1])
            print(f"Using camera index: {camera_index}")
        except ValueError:
            print(f"Invalid camera index '{sys.argv[1]}', using default (0)")
    
    # Initialize board
    print("Connecting to robot controller...")
    try:
        board = rrc.Board()
        time.sleep(0.5)
    except Exception as e:
        print(f"\nERROR: Failed to connect to robot controller: {e}")
        print("Make sure /dev/rrc is available and you have permissions")
        sys.exit(1)
    
    # Create tracker
    try:
        tracker = RedBallTracker(board, camera_index=camera_index)
    except RuntimeError as e:
        print(f"\nFailed to initialize tracker: {e}")
        sys.exit(1)
    
    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, tracker.handle_shutdown)
    signal.signal(signal.SIGTERM, tracker.handle_shutdown)
    
    # Run tracking
    tracker.run()


if __name__ == "__main__":
    main()
