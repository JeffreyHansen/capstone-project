#!/bin/bash
# Stop camera-using processes to free up /dev/video0

echo "=== Stopping Camera Processes ==="
echo ""

# Function to stop docker containers
stop_docker() {
    echo "1. Checking for Docker containers..."
    if command -v docker &> /dev/null; then
        containers=$(docker ps -q)
        if [ -n "$containers" ]; then
            echo "   Found running containers. Stopping all..."
            docker stop $(docker ps -q)
            echo "   ✓ Docker containers stopped"
        else
            echo "   No running containers"
        fi
    else
        echo "   Docker not found"
    fi
    echo ""
}

# Function to kill processes using video device
kill_video_users() {
    echo "2. Checking processes using /dev/video*..."
    if command -v fuser &> /dev/null; then
        if sudo fuser /dev/video* 2>/dev/null; then
            echo "   Found processes using camera. Kill them? (y/n)"
            read -r response
            if [[ "$response" == "y" ]]; then
                sudo fuser -k /dev/video*
                echo "   ✓ Processes killed"
            else
                echo "   Skipped"
            fi
        else
            echo "   No processes using camera"
        fi
    else
        echo "   fuser command not available"
    fi
    echo ""
}

# Function to stop ROS2 nodes
stop_ros2() {
    echo "3. Checking for ROS2 processes..."
    ros_pids=$(ps aux | grep -E "ros2|usb_cam" | grep -v grep | awk '{print $2}')
    if [ -n "$ros_pids" ]; then
        echo "   Found ROS2 processes: $ros_pids"
        echo "   Kill them? (y/n)"
        read -r response
        if [[ "$response" == "y" ]]; then
            kill $ros_pids 2>/dev/null
            echo "   ✓ ROS2 processes killed"
        else
            echo "   Skipped"
        fi
    else
        echo "   No ROS2 processes found"
    fi
    echo ""
}

# Main execution
echo "This script will help you stop processes using the camera."
echo "Choose an option:"
echo "  1) Stop Docker containers only"
echo "  2) Stop ROS2 processes only"
echo "  3) Kill all video device users (aggressive)"
echo "  4) Do all of the above"
echo "  5) Cancel"
echo ""
read -p "Enter choice (1-5): " choice

case $choice in
    1)
        stop_docker
        ;;
    2)
        stop_ros2
        ;;
    3)
        kill_video_users
        ;;
    4)
        stop_docker
        stop_ros2
        kill_video_users
        ;;
    5)
        echo "Cancelled"
        exit 0
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac

echo "=== Done ==="
echo ""
echo "Checking if camera is now free..."
sleep 1

if sudo fuser /dev/video0 2>/dev/null; then
    echo "⚠ Camera still in use!"
    sudo lsof /dev/video0 2>/dev/null || echo "Cannot determine what's using it"
else
    echo "✓ Camera is free! You can now run the tracker."
fi
