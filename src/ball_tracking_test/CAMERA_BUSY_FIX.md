# Camera Busy Error - Quick Fix Guide

## The Problem

Your error shows:
```
Device '/dev/video0' is busy
```

This means **another process is using the camera**. Common culprits:

1. **ROS2 Docker Container** - Most likely! The `usb_cam` node from ROS2
2. **ROS2 System Service** - A background service running the camera driver
3. **Another OpenCV/GStreamer process**

## Quick Solutions

### Solution 1: Use the Auto-Fix Script (Recommended) ✅

```bash
cd /Users/jeffrey/Documents/Capstone/capstone-project/src/board_demo

# Run the interactive script
./free_camera.sh

# Follow the prompts to stop Docker/ROS2 processes
```

### Solution 2: Manually Check and Stop

```bash
# 1. Check what's using the camera
./check_camera.sh

# 2. If it shows Docker containers, stop them:
docker ps                    # See running containers
docker stop <container_name> # Stop the specific one
# or
docker stop $(docker ps -q)  # Stop ALL containers

# 3. If it shows ROS2 processes:
pkill -f usb_cam
pkill -f ros2

# 4. Nuclear option (kills everything using camera):
sudo fuser -k /dev/video0
```

### Solution 3: Use a Different Camera Device

If you have multiple cameras or the busy one isn't needed:

```bash
# Check available cameras
ls -l /dev/video*

# Run tracker with different camera (e.g., video1)
python3 red_ball_tracker.py 1
```

## Step-by-Step: Most Common Case

If you're running the ROS2 system in Docker:

```bash
# Step 1: See what's running
docker ps

# Step 2: Stop the container (example name)
docker stop turbopi_ros2

# Step 3: Verify camera is free
ls -l /dev/video0
sudo lsof /dev/video0    # Should return nothing

# Step 4: Run the tracker
python3 red_ball_tracker.py
```

## Prevention

To prevent this issue:

### Option A: Don't run ROS2 and standalone tracker simultaneously
- Stop Docker before running standalone scripts
- Stop standalone scripts before running Docker

### Option B: Integrate with ROS2 (Advanced)
Instead of standalone, subscribe to ROS2 camera topic:
- Camera already running in ROS2 container
- Subscribe to `/image_raw` topic
- No camera conflict!

### Option C: Use different cameras
- Plug in second USB camera
- ROS2 uses `/dev/video0`
- Tracker uses `/dev/video1`

## Verifying the Fix

After stopping processes:

```bash
# This should show NO output (camera is free)
sudo fuser /dev/video0

# This should show the device but no process using it
ls -l /dev/video0

# Now run the tracker
python3 red_ball_tracker.py
```

## If Still Not Working

1. **Reboot the robot**
   ```bash
   sudo reboot
   ```

2. **Check camera permissions**
   ```bash
   sudo chmod 666 /dev/video0
   ls -l /dev/video0  # Should show rw-rw-rw-
   ```

3. **Test camera directly**
   ```bash
   # Install v4l-utils if not present
   sudo apt-get install v4l-utils
   
   # Test camera
   v4l2-ctl --device=/dev/video0 --all
   ```

4. **Try simple OpenCV test**
   ```python
   python3 -c "import cv2; cap = cv2.VideoCapture(0); print('Success!' if cap.isOpened() else 'Failed'); cap.release()"
   ```

## Understanding the Error in Detail

Your full error breakdown:

1. **GStreamer warnings** - OpenCV trying GStreamer backend first
   - `Device '/dev/video0' is busy` ← **THE MAIN ISSUE**

2. **V4L2 warning** - OpenCV trying V4L2 backend second
   - `can't open camera by index` ← Same device busy error

3. **Failed to capture frame** - Script couldn't initialize camera
   - Because camera was never opened successfully

## Updated Tracker Features

The tracker now has:
- **Auto-retry**: Tries video0, video1, video2 automatically
- **Better errors**: Tells you exactly what's wrong
- **CLI argument**: Can specify camera: `python3 red_ball_tracker.py 1`

## Need More Help?

Check system logs:
```bash
# Check for camera-related errors
dmesg | grep -i video

# Check running services
systemctl list-units | grep -i camera

# Check Docker logs
docker logs <container_name>
```
