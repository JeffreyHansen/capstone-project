#!/bin/bash
# Camera diagnostic script - find what's using the camera

echo "=== Camera Device Check ==="
echo ""

# Check if camera device exists
echo "1. Checking camera devices:"
ls -l /dev/video* 2>/dev/null || echo "   No video devices found!"
echo ""

# Check what processes are using the camera
echo "2. Processes using video devices:"
if command -v lsof &> /dev/null; then
    sudo lsof /dev/video* 2>/dev/null || echo "   No processes found using lsof"
else
    echo "   lsof not installed, checking with fuser..."
    sudo fuser /dev/video* 2>/dev/null || echo "   No processes found"
fi
echo ""

# Check for running Docker containers
echo "3. Running Docker containers:"
if command -v docker &> /dev/null; then
    docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}" 2>/dev/null || echo "   No Docker containers running or no permission"
else
    echo "   Docker not installed or not in PATH"
fi
echo ""

# Check for ROS2 processes
echo "4. ROS2 processes:"
ps aux | grep -E "ros2|usb_cam|camera" | grep -v grep || echo "   No ROS2/camera processes found"
echo ""

# Check for v4l2 processes
echo "5. V4L2 (Video4Linux) processes:"
ps aux | grep -E "v4l2|gstreamer" | grep -v grep || echo "   No V4L2 processes found"
echo ""

echo "=== Suggestions ==="
echo ""
echo "If processes were found above:"
echo "  • Stop Docker container: docker stop <container_name>"
echo "  • Kill specific process: kill <PID>"
echo "  • Kill all camera users: sudo fuser -k /dev/video0"
echo ""
echo "To run the tracker, first stop anything using the camera!"
