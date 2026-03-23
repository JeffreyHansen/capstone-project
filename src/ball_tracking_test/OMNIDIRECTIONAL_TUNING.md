# Omnidirectional Robot Tuning Guide

## Major Updates Applied ✅

### 1. **Omnidirectional Movement** (Mecanum/Omni Wheels)
- Added `move_robot_omni()` with **3-axis control**: forward, strafe, rotation
- Smart strategy: Rotates when ball far off-center, strafes when near center
- Prevents mixing rotation + strafe (reduces confusion)

### 2. **Motion Blur Reduction**
- **Reduced max speed**: 60 → 35 (40% reduction)
- **Camera settings**: Faster shutter speed (lower exposure)
- **Increased gain**: Compensate for less light

### 3. **Anti-Oscillation Fixes**
- **Detection smoothing**: Moving average over 5 frames
- **Less aggressive PID**: Reduced P-values by 50-70%
- **Larger deadzones**: Ignore small errors (25-35 pixels)
- **Separate rotation deadzone**: Prevents jittery spinning

### 4. **Visual Feedback**
- Red circle = raw detection
- Green circle = smoothed detection (what robot uses)
- Shows both raw and smoothed values in overlay

---

## Testing & Calibration

### Step 1: Test Motor Direction (CRITICAL!)

The motor signs in `move_robot_omni()` may need adjustment for your robot's wiring.

```python
# Current motor mapping (in move_robot_omni):
self.set_motor_speeds(-m1, m2, -m3, m4)
#                      ^   ^   ^   ^
#                      M1  M2  M3  M4
```

**Test each movement type:**

```bash
# Run tracker and observe:
python3 red_ball_tracker.py
```

**Expected behavior when ball moves:**

| Ball Position | Robot Should | If Robot Does Opposite | Fix |
|---------------|--------------|------------------------|-----|
| **Left of center** | Rotate/strafe LEFT | Rotates/strafes RIGHT | Invert rotation or strafe signs |
| **Right of center** | Rotate/strafe RIGHT | Rotates/strafes LEFT | Invert rotation or strafe signs |
| **Far away (small)** | Move FORWARD | Moves BACKWARD | Invert forward signs |
| **Too close (large)** | Move BACKWARD | Moves FORWARD | Invert forward signs |

**Quick motor test script:**

```python
# Add to red_ball_tracker.py for testing
def test_motors(board):
    """Test each movement type independently"""
    tracker = RedBallTracker(board)
    
    print("Testing FORWARD...")
    tracker.move_robot_omni(30, 0, 0)  # Should move forward
    time.sleep(2)
    tracker.stop_robot()
    time.sleep(1)
    
    print("Testing STRAFE RIGHT...")
    tracker.move_robot_omni(0, 30, 0)  # Should strafe right
    time.sleep(2)
    tracker.stop_robot()
    time.sleep(1)
    
    print("Testing ROTATE CLOCKWISE...")
    tracker.move_robot_omni(0, 0, 30)  # Should rotate clockwise
    time.sleep(2)
    tracker.stop_robot()
    
    print("Test complete!")

# Run with: python3 -c "import red_ball_tracker as rbt; import ros_robot_controller_sdk as rrc; rbt.test_motors(rrc.Board())"
```

### Step 2: Adjust Motor Signs

If movements are wrong, modify the motor signs in `move_robot_omni()`:

```python
# Example fixes:
# If forward is backward, change:
self.set_motor_speeds(m1, -m2, m3, -m4)  # Inverted forward direction

# If strafe is reversed, change:
self.set_motor_speeds(m1, -m2, m3, -m4)  # Inverted strafe direction

# If rotation is reversed, change:
self.set_motor_speeds(m1, m2, m3, m4)  # Removed rotation inversions

# You may need a combination - test each movement separately!
```

### Step 3: Fine-Tune Speed & PID

After motor directions are correct:

#### Reduce Speed Further If Still Blurry:
```python
self.motor_max_speed = 25  # Even slower (was 35)
self.motor_min_speed = 12  # Lower threshold
```

#### Increase Smoothing If Oscillating:
```python
self.history_size = 8  # More averaging (was 5)
```

#### Adjust PID if Too Slow/Fast:
```python
# If robot is too sluggish:
self.chassis_rot_pid = PID(P=0.10, I=0.002, D=0.01)  # Increase P

# If still oscillating:
self.chassis_rot_pid = PID(P=0.04, I=0.001, D=0.012)  # Decrease P, increase D
```

#### Adjust Deadzones:
```python
# If robot is too jittery (tiny movements):
self.rotation_deadzone = 50  # Larger (was 30)

# If robot doesn't react enough:
self.rotation_deadzone = 20  # Smaller (was 30)
```

### Step 4: Camera Exposure Tuning

If exposure settings don't work (camera doesn't support them):

```python
# Try these alternatives:
self.camera.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # Manual exposure
self.camera.set(cv2.CAP_PROP_EXPOSURE, -5)  # Adjust this value
self.camera.set(cv2.CAP_PROP_BRIGHTNESS, 50)  # If too dark
```

Check what your camera supports:
```python
# Add this after camera setup to see supported properties:
print(f"Exposure: {self.camera.get(cv2.CAP_PROP_EXPOSURE)}")
print(f"FPS: {self.camera.get(cv2.CAP_PROP_FPS)}")
print(f"Auto-exposure: {self.camera.get(cv2.CAP_PROP_AUTO_EXPOSURE)}")
```

---

## Understanding Omnidirectional Control

### Mecanum Wheel Movement Equations

For a typical mecanum wheel robot:

```
       Front
    [M1]  [M2]
     /\    /\    (Wheel rollers orientation)
    [M3]  [M4]
```

**Movement formulas:**
```python
# Forward movement (all wheels same direction)
forward: M1=+, M2=+, M3=+, M4=+

# Strafe right (diagonal wheels opposite)
strafe_right: M1=+, M2=-, M3=-, M4=+

# Rotate clockwise (left/right wheels opposite)
rotate_cw: M1=+, M2=-, M3=+, M4=-

# Combined (additive):
M1 = forward + strafe + rotation
M2 = forward - strafe - rotation
M3 = forward - strafe + rotation
M4 = forward + strafe - rotation
```

### Strategy: Rotation vs Strafe

```python
if abs(x_error) > 100:
    # Ball FAR off-center (>100px): ROTATE to face it
    # Why? Rotation is faster for large corrections
    use_rotation = True
else:
    # Ball NEAR center (<100px): STRAFE to fine-tune
    # Why? Strafe keeps robot facing forward while centering
    use_strafe = True
```

This prevents: Rotation + strafe fighting each other!

---

## Troubleshooting

### Problem: Robot spins in place
**Cause**: Rotation PID too aggressive or motor signs wrong
**Fix**:
1. Check motor test - rotation should be smooth
2. Reduce `chassis_rot_pid` P-value
3. Increase `rotation_deadzone`

### Problem: Robot still oscillates
**Cause**: Detection noise or PID too reactive
**Fix**:
1. Increase `history_size` to 8-10
2. Reduce all PID P-values by 50%
3. Increase D-values for damping
4. Increase deadzones

### Problem: Ball still blurry
**Cause**: Motors too fast or camera exposure too long
**Fix**:
1. Reduce `motor_max_speed` to 25 or 20
2. Lower exposure: `cv2.CAP_PROP_EXPOSURE, -7`
3. Ensure good lighting (camera needs more light with fast shutter)

### Problem: Robot doesn't move forward/backward
**Cause**: Distance error calculation or motor signs
**Fix**:
1. Check if `distance_error` calculation is correct
2. Test forward movement independently
3. Verify ball radius is being detected (check debug overlay)

### Problem: Robot moves backward when should go forward
**Cause**: Motor orientation or PID sign
**Fix**:
```python
# In update_chassis_tracking(), change:
forward_speed = -self.chassis_y_pid.update(distance_error)
# to:
forward_speed = self.chassis_y_pid.update(distance_error)
```

### Problem: Detection history shows 0 frames
**Cause**: Ball not being detected (blur or wrong color)
**Fix**:
1. Check "Detection Mask" window - should show white blob
2. Adjust red color ranges (HSV values)
3. Reduce motor speed further
4. Improve lighting

---

## Advanced: Kalman Filter (Optional)

If moving average isn't enough, implement Kalman filter:

```python
import cv2

class KalmanBallTracker:
    def __init__(self):
        self.kalman = cv2.KalmanFilter(4, 2)  # 4 states (x,y,vx,vy), 2 measurements (x,y)
        self.kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                                   [0, 1, 0, 0]], np.float32)
        self.kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                                  [0, 1, 0, 1],
                                                  [0, 0, 1, 0],
                                                  [0, 0, 0, 1]], np.float32)
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 0.03
    
    def update(self, x, y):
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        self.kalman.correct(measurement)
        prediction = self.kalman.predict()
        return prediction[0][0], prediction[1][0]
```

---

## Performance Checklist

After tuning, verify:

- [ ] Robot moves forward when ball is far (small in frame)
- [ ] Robot moves backward when ball is close (large in frame)
- [ ] Robot rotates to face ball when off-center >100px
- [ ] Robot strafes to center ball when off-center <100px
- [ ] No oscillation (back-and-forth spinning)
- [ ] Ball detection is smooth (green circle stable)
- [ ] Motors don't stall (minimum speed threshold works)
- [ ] Camera image is not blurry during movement
- [ ] Detection history shows 3-5 frames consistently

---

## Quick Reference: Key Parameters

```python
# Speed limits (adjust first!)
motor_max_speed = 35        # Lower = less blur, slower response
motor_min_speed = 15        # Lower = smoother slow movements

# PID tuning (adjust second!)
chassis_rot_pid = PID(P=0.08, I=0.002, D=0.01)  # P: response, D: damping
chassis_x_pid = PID(P=0.06, I=0.001, D=0.008)   # Strafe
chassis_y_pid = PID(P=0.05, I=0.001, D=0.005)   # Forward

# Smoothing (adjust third!)
history_size = 5            # Higher = smoother, slower response

# Deadzones (adjust last!)
rotation_deadzone = 30      # Pixels to ignore for rotation
chassis_deadzone = 35       # Pixels to ignore for strafe
servo_deadzone = 25         # Pixels to ignore for gimbal

# Strategy threshold
if abs(x_error) > 100:      # Switch between rotate/strafe
```

Start conservative (current values) and increase speed only after confirming no oscillation!
