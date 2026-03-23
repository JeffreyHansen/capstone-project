# Migration Guide: Standalone Tracker → ROS2 Soccer System

## Current State
- **Standalone script**: `red_ball_tracker.py` (works, but limited)
- **Detects**: Only red color (basic HSV filtering)
- **Limitation**: Can't scale to soccer use case

## Target Architecture: ROS2 Soccer Robot

### Phase 1: Keep Standalone for Testing ✅
**What**: Use current `red_ball_tracker.py` for quick hardware testing
**When**: 
- Testing camera/servo/motor hardware
- Debugging PID parameters
- Quick demos

**How**: 
```bash
# Stop Docker first
docker stop $(docker ps -q)

# Run standalone
python3 red_ball_tracker.py
```

### Phase 2: Integrate with ROS2 (Recommended for Soccer) 🎯

#### Step 1: Use Existing YOLOv11 for Object Detection

Your workspace already has `src/yolov11_detect/`! 

**Train/Configure for Soccer Objects:**
```bash
# Configure YOLO to detect:
# - Class 0: soccer_ball
# - Class 1: robot_player  
# - Class 2: goal_blue
# - Class 3: goal_yellow
```

**Existing YOLO Node** (check `src/yolov11_detect/yolov11_detect/yolov11_node.py`):
- Already subscribes to camera
- Already publishes detections
- You just need proper model weights

#### Step 2: Create Soccer Strategy Node

Create new package: `src/soccer_strategy/`

```python
# src/soccer_strategy/soccer_strategy/strategy_node.py

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point
from std_msgs.msg import String

class SoccerStrategy(Node):
    def __init__(self):
        super().__init__('soccer_strategy')
        
        # Subscribe to YOLO detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10
        )
        
        # Publish target for tracker
        self.target_pub = self.create_publisher(
            Point,
            '/target_position',
            10
        )
        
        # Publish game state
        self.state_pub = self.create_publisher(
            String,
            '/game_state',
            10
        )
        
        self.mode = 'attack'  # or 'defend'
        
    def detection_callback(self, msg):
        """Process YOLO detections and decide strategy"""
        
        # Extract detected objects
        ball = None
        opponent = None
        our_goal = None
        their_goal = None
        
        for detection in msg.detections:
            class_id = detection.results[0].id
            bbox = detection.bbox
            
            if class_id == 0:  # soccer_ball
                ball = bbox.center
            elif class_id == 1:  # robot_player
                opponent = bbox.center
            elif class_id == 2:  # goal_blue
                our_goal = bbox.center
            elif class_id == 3:  # goal_yellow
                their_goal = bbox.center
        
        # Decision making
        if ball is not None:
            # Strategy logic
            if self.should_attack(ball, opponent, their_goal):
                self.mode = 'attack'
                target = ball  # Chase the ball
            else:
                self.mode = 'defend'
                target = self.get_defensive_position(ball, our_goal)
            
            # Publish target for tracker
            target_msg = Point()
            target_msg.x = target.x
            target_msg.y = target.y
            self.target_pub.publish(target_msg)
            
            # Publish game state
            state_msg = String()
            state_msg.data = self.mode
            self.state_pub.publish(state_msg)
    
    def should_attack(self, ball, opponent, goal):
        """Decide whether to attack or defend"""
        # Example logic (customize this!)
        
        # Attack if ball is closer to opponent's goal
        if goal is not None:
            distance_to_goal = abs(ball.x - goal.x)
            if distance_to_goal < 200:  # pixels
                return True
        
        # Defend if opponent is close to ball
        if opponent is not None:
            distance_to_opponent = abs(ball.x - opponent.x)
            if distance_to_opponent < 100:
                return False
        
        return True  # Default: attack
    
    def get_defensive_position(self, ball, goal):
        """Calculate defensive position between ball and goal"""
        pos = Point()
        if goal is not None:
            # Position between ball and goal
            pos.x = (ball.x + goal.x) / 2
            pos.y = (ball.y + goal.y) / 2
        else:
            pos.x = ball.x
            pos.y = ball.y
        return pos

def main():
    rclpy.init()
    node = SoccerStrategy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Step 3: Modify Existing Tracking Node

Modify `src/app/app/tracking.py` to accept target from strategy node:

```python
# Add subscription to target position
self.target_sub = self.create_subscription(
    Point,
    '/target_position',
    self.target_callback,
    10
)

def target_callback(self, msg):
    """Receive target from strategy node"""
    self.target_x = msg.x
    self.target_y = msg.y
    # Use this instead of color detection
```

### Phase 3: Docker Configuration for Soccer

Update your Docker setup to run all nodes:

```yaml
# docker-compose.yml (example)
version: '3'
services:
  soccer_robot:
    image: your_robot_image
    privileged: true
    devices:
      - /dev/video0:/dev/video0
      - /dev/rrc:/dev/rrc
    network_mode: host
    volumes:
      - ./src:/workspace/src
    command: >
      bash -c "
        source /workspace/install/setup.bash &&
        ros2 launch soccer_bringup soccer_robot.launch.py
      "
```

```python
# src/soccer_bringup/launch/soccer_robot.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB Camera
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[...]
        ),
        
        # YOLO Detection
        Node(
            package='yolov11_detect',
            executable='yolov11_node',
            name='object_detector',
            parameters=[{
                'model_path': '/workspace/models/soccer_yolo.pt',
                'classes': ['ball', 'player', 'goal_blue', 'goal_yellow']
            }]
        ),
        
        # Soccer Strategy
        Node(
            package='soccer_strategy',
            executable='strategy_node',
            name='soccer_brain'
        ),
        
        # Object Tracking
        Node(
            package='app',
            executable='tracking_node',
            name='tracker'
        ),
        
        # Motor Controller
        Node(
            package='ros_robot_controller',
            executable='controller_node',
            name='motor_controller'
        ),
    ])
```

## Comparison: Standalone vs ROS2 for Soccer

| Feature | Standalone Script | ROS2 System |
|---------|------------------|-------------|
| **Simple ball tracking** | ✅ Perfect | ⚠️ Overkill |
| **Multiple object detection** | ❌ Hard | ✅ Easy (YOLO) |
| **Strategy/AI decision** | ❌ Very hard | ✅ Separate node |
| **Multi-robot coordination** | ❌ Impossible | ✅ DDS networking |
| **Development speed** | ✅ Fast for simple | ⚠️ Setup time |
| **Scalability** | ❌ Limited | ✅ Excellent |
| **Debugging** | ✅ Simple | ⚠️ More complex |
| **Team collaboration** | ❌ Difficult | ✅ Good |
| **Hardware testing** | ✅ Ideal | ⚠️ Need Docker |
| **Production soccer** | ❌ No | ✅ Yes |

## Recommendation: Hybrid Approach 🎯

### Use Standalone For:
1. **Hardware validation** - Test motors, servos, camera
2. **PID tuning** - Quick iterations on control parameters
3. **Quick demos** - Show basic tracking to stakeholders
4. **Learning** - Understand the concepts

### Use ROS2 For:
1. **Soccer gameplay** - Your actual competition system
2. **ML integration** - YOLO object detection
3. **Strategy development** - Attack/defend logic
4. **Multi-robot** - Coordination between teammates
5. **Production** - Final tournament code

## Next Steps

1. **Short term** (this week):
   - ✅ Keep using `red_ball_tracker.py` for hardware testing
   - ✅ Tune PID parameters with standalone script
   - ✅ Test motor directions, servo ranges

2. **Medium term** (next 2 weeks):
   - 📦 Train YOLO model for soccer objects (ball, robots, goals)
   - 🧠 Create `soccer_strategy` package
   - 🔧 Modify existing `tracking.py` to use YOLO detections

3. **Long term** (before competition):
   - 🐳 Finalize Docker setup with all nodes
   - 🤖 Test multi-robot communication
   - 🏆 Develop game strategies (attack/defend logic)
   - 🧪 Integration testing

## Example Workflow

```bash
# Development/Testing (standalone)
docker stop $(docker ps -q)          # Free the camera
python3 red_ball_tracker.py          # Test tracking

# Soccer Mode (ROS2)
docker-compose up -d                  # Start ROS2 system
# Or
ros2 launch soccer_bringup soccer_robot.launch.py
```

## Resources in Your Workspace

- **YOLOv11**: `src/yolov11_detect/` - Already set up!
- **Tracking**: `src/app/app/tracking.py` - Good reference
- **Motor control**: `src/driver/ros_robot_controller/`
- **Camera**: `src/peripherals/launch/usb_cam.launch.py`

The foundation is already there - you just need to connect the pieces!
