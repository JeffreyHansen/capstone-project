# Red Ball Tracker - Setup and Usage Guide

## Overview
This script tracks a red ball using the robot's camera and follows it with both:
- **Camera gimbal** (PWM servos 1 & 2) - keeps ball centered in view
- **Chassis motors** (4 DC motors) - moves robot to maintain distance from ball

## Hardware Requirements
- Hiwonder Raspberry Pi Robot with expansion board
- USB camera connected to `/dev/video0`
- PWM servos 1 & 2 installed (camera gimbal)
- 4 DC motors connected to motor ports 1-4

## Software Requirements
```bash
# Install dependencies (if not already installed)
sudo apt-get update
sudo apt-get install -y python3-opencv python3-numpy python3-serial
```

## How to Run

### Option 1: Direct Execution (Recommended)
Run directly on the robot via terminal:

```bash
# Navigate to the board_demo directory
cd /Users/jeffrey/Documents/Capstone/capstone-project/src/board_demo

# Make the script executable
chmod +x red_ball_tracker.py

# Run the script
python3 red_ball_tracker.py
```

### Option 2: Running in Background
```bash
# Run in background with output logging
python3 red_ball_tracker.py > tracker.log 2>&1 &

# Check if running
ps aux | grep red_ball_tracker

# Stop the process
pkill -f red_ball_tracker.py
```

## Docker vs Terminal?

**Use Terminal (python3) - This is the recommended approach because:**

1. ✅ **Direct hardware access** - needs `/dev/video0`, `/dev/rrc` serial port
2. ✅ **Simple deployment** - no container overhead
3. ✅ **Real-time performance** - no virtualization layer
4. ✅ **Easy debugging** - can see camera feed directly via X11

**Docker would add complexity:**
- ❌ Needs privileged mode for hardware access
- ❌ Needs X11 forwarding for camera display
- ❌ Additional container management overhead
- ❌ Potential latency issues

## Controls

### During Execution
- **'q' key** - Quit the program (when video window is focused)
- **Ctrl+C** - Stop the program from terminal

### Adjustable Parameters
Edit these variables in the script to tune performance:

```python
# Color detection (lines 56-59) - adjust for your red ball
self.lower_red1 = np.array([0, 100, 100])
self.upper_red1 = np.array([10, 255, 255])

# Servo PID (line 77-78) - adjust for smoother/faster gimbal movement
self.servo_x_pid = PID(P=0.3, I=0.05, D=0.01)

# Chassis PID (line 85-86) - adjust for smoother/faster robot movement
self.chassis_x_pid = PID(P=0.15, I=0.01, D=0.005)

# Motor speeds (line 81-82)
self.motor_max_speed = 60  # Maximum speed (0-100)
self.motor_min_speed = 25  # Minimum speed to move

# Target distance (line 93)
self.target_ball_radius = 80  # Desired ball size in pixels
```

## How It Works

### 1. **Ball Detection**
- Captures video from USB camera at 640x480
- Converts to HSV color space
- Creates mask for red colors
- Finds largest red contour (the ball)

### 2. **Gimbal Tracking**
- PID controller keeps ball centered in camera view
- Servo 1 (Y-axis): vertical tilt (1200-1900)
- Servo 2 (X-axis): horizontal pan (800-2200)

### 3. **Chassis Following**
- **Rotation**: PID based on ball's horizontal position
  - Ball left of center → robot rotates left
  - Ball right of center → robot rotates right
- **Forward/Backward**: PID based on ball's size
  - Ball too small (far) → robot moves forward
  - Ball too large (close) → robot moves backward

### 4. **Motor Control**
```
Motor Layout (top view):
    [1]  [2]
     \\  //
     [CAR]
     //  \\
    [3]  [4]

Forward: Motors alternate signs due to mounting
Rotation: Left motors vs right motors opposite directions
```

## Troubleshooting

### Camera not found
```bash
# Check if camera is connected
ls -l /dev/video*

# Test camera
python3 -c "import cv2; print(cv2.VideoCapture(0).read()[0])"
```

### Robot controller not found
```bash
# Check serial connection
ls -l /dev/rrc

# Check permissions
sudo chmod 666 /dev/rrc
```

### Red ball not detected
1. Adjust red color ranges in HSV (lines 56-59)
2. Ensure good lighting conditions
3. Check minimum ball radius threshold (line 90)
4. View "Detection Mask" window to see what's being detected

### Robot doesn't move
1. Check battery level
2. Verify motor connections (ports 1-4)
3. Increase `motor_min_speed` if motors don't overcome friction
4. Check if `chassis_deadzone` is too large

### Jittery movement
1. Reduce PID P-values for smoother response
2. Increase D-values for damping
3. Increase deadzone thresholds

## Safety Notes

⚠️ **Important:**
- Robot will move autonomously - ensure clear space
- Keep emergency stop ready (Ctrl+C)
- Test in safe environment first
- Monitor battery level
- Ensure ball is visible and distinct from background

## Example Session

```bash
ubuntu@turbopi:~/board_demo$ python3 red_ball_tracker.py

**********************************************************
****Red Ball Tracking with Camera Gimbal and Chassis****
**********************************************************
...
Red Ball Tracker initialized!
- Camera resolution: 640x480
- Processing resolution: 320x240
- Press 'q' to quit

Starting red ball tracking...
Tracking parameters:
  - Minimum ball radius: 10 pixels
  - Target ball radius: 80 pixels
  - Servo deadzone: 15 pixels
  - Chassis deadzone: 20 pixels

[Tracking begins - robot follows red ball]

^C
Interrupted by user
Cleaning up...
Cleanup complete
```

## Integration with ROS2

If you want to integrate this with ROS2 later, you can:
1. Convert to a ROS2 node
2. Subscribe to camera topic instead of cv2.VideoCapture
3. Publish to existing motor/servo topics
4. See `src/app/app/tracking.py` for ROS2 example

## License
Based on Hiwonder robot SDK examples
