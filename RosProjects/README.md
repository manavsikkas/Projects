# ROS2 Projects

This directory contains various ROS2 (Robot Operating System 2) projects.

## Projects

### 1. TurtleBot Gesture Control
A ROS2 package for controlling TurtleBot3 using hand gestures.

**Features:**
- Hand gesture detection using computer vision
- Robot controller for TurtleBot3 movement
- IMU simulator for sensor data
- GNSS mock for location services
- RMSE calculator for performance metrics

**Key Components:**
- `hand_gesture_detector.py`: Detects and interprets hand gestures
- `robot_controller.py`: Controls TurtleBot3 based on gesture commands
- `imu_simulator.py`: Simulates IMU sensor data
- `gnss_mock.py`: Provides mock GNSS data
- `rmse_calculator.py`: Calculates root mean square error for evaluation

**Launch Files:**
- `turtlebot_launch.py`: Main launch file for the complete system

**Models:**
- TurtleBot3 Waffle model (SDF format)
- Custom TurtleBot model

**Installation:**
```bash
# Clone the repository
cd ~/ros2_ws/src
cp -r /path/to/turtlebot_gesture_control .

# Build the package
cd ~/ros2_ws
colcon build --packages-select turtlebot

# Source the setup file
source install/setup.bash

# Launch the system
ros2 launch turtlebot turtlebot_launch.py
```

**Requirements:**
- ROS2 (Humble/Iron/Rolling)
- Python 3.8+
- OpenCV (for gesture detection)
- Gazebo (for simulation)