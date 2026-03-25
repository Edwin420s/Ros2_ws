# KIRINYAGA UNIVERSITY - ECE 2318 ROBOTICS
# ROS 2 TURTLEBOT3 SIMULATION PROJECT

## Project Team
- **EE100/G/19629/23** - NEVIL KIPTOO ROP
- **EE100/G/21017/23** - CLEMENT THAIRU KIBOCHA  
- **EE100/G/14624/21** - IRENE KURIA
- **EE100/G/19111/23** - KEVIN KIPROTICH
- **EE100G/G/21048/23** - ISACK OGINGA
- **EE100/G/20564/23** - THUO JOSEPH WAINAINA
- **EE100/G/17974/23** - GERSON KIPSANG

## Introduction

This project implements a complete ROS 2 simulation of a Turtlebot3 Burger mobile robot using Gazebo and RViz2. The simulation demonstrates all core ROS 2 concepts including nodes, topics, messages, publishers, subscribers, services, actions, and parameters.

## Features

### 🤖 Robot Model
- **Complete URDF model** of Turtlebot3 Burger
- **Differential drive** kinematics with realistic physics
- **Lidar sensor** (360° scanning, 10Hz update)
- **IMU sensor** with noise and covariance
- **Proper inertial properties** and collision geometry

### 🎮 Motion Control
- **/cmd_vel topic** for velocity commands (geometry_msgs/Twist)
- **Multiple motion patterns**: square, circle, figure-8
- **Smooth velocity transitions** with acceleration limits
- **Emergency stop** functionality

### 📊 Visualization
- **RViz2 integration** with custom configuration
- **Robot model display** with proper TF hierarchy
- **Real-time LiDAR scan** visualization
- **Odometry trail** and coordinate frames

### 🔧 ROS 2 Core Elements
- **Nodes**: velocity_controller, teleop_node, service_server, action_server
- **Topics**: /cmd_vel, /scan, /odom, /imu, /robot_status
- **Services**: emergency_stop, set_linear_speed, get_status, reset_odometry
- **Actions**: navigate_to_position, patrol_area, scan_area
- **Parameters**: Dynamic speed control, behavior modification

## Installation and Setup

### Prerequisites
```bash
# ROS 2 Humble (required)
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3*
sudo apt install python3-colcon-common-extensions
```

### Build the Project
```bash
# Navigate to ROS 2 workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select turtlebot3_simulation

# Source the workspace
source install/setup.bash
```

## Running the Simulation

### 1. Complete Simulation (Gazebo + RViz2)
```bash
ros2 launch turtlebot3_simulation turtlebot3_simulation.launch.py
```

### 2. Gazebo Only
```bash
ros2 launch turtlebot3_simulation gazebo_only.launch.py
```

### 3. RViz2 Only
```bash
ros2 launch turtlebot3_simulation rviz_only.launch.py
```

## Robot Control

### Automatic Motion Control
```bash
# Start velocity controller with square pattern
ros2 run turtlebot3_simulation velocity_controller.py

# Change motion pattern dynamically
ros2 param set /velocity_controller motion_pattern "circle"

# Adjust speeds
ros2 param set /velocity_controller max_linear_speed 0.3
ros2 param set /velocity_controller max_angular_speed 1.0
```

### Manual Teleoperation
```bash
# Start teleoperation node
ros2 run turtlebot3_simulation teleop_node.py

# Controls:
# w/x - increase/decrease linear velocity
# a/d - increase/decrease angular velocity  
# s   - stop robot
# q   - quit
```

### Keyboard Teleoperation (Alternative)
```bash
# Use standard ROS 2 teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Services and Actions

### Available Services
```bash
# Emergency stop
ros2 service call /emergency_stop std_srvs/SetBool "{data: true}"

# Set speed limits
ros2 service call /set_linear_speed example_interfaces/SetFloat64 "{data: 0.3}"
ros2 service call /set_angular_speed example_interfaces/SetFloat64 "{data: 1.0}"

# Get robot status
ros2 service call /get_status std_srvs/Trigger

# Reset odometry
ros2 service call /reset_odometry std_srvs/Trigger
```

### Service Server
```bash
# Start service server
ros2 run turtlebot3_simulation service_server.py
```

### Action Server
```bash
# Start action server
ros2 run turtlebot3_simulation action_server.py
```

## Topics and Messages

### Key Topics
```bash
# List all topics
ros2 topic list

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor LiDAR scans
ros2 topic echo /scan

# Monitor odometry
ros2 topic echo /odom

# Monitor IMU data
ros2 topic echo /imu
```

### Topic Information
```bash
# Get topic info
ros2 topic info /cmd_vel
ros2 topic info /scan

# Monitor publishing rates
ros2 topic hz /cmd_vel
ros2 topic hz /scan
```

## Parameter Management

### View Parameters
```bash
# List all parameters
ros2 param list /velocity_controller

# Get parameter values
ros2 param get /velocity_controller max_linear_speed
ros2 param get /velocity_controller motion_pattern
```

### Dynamic Parameter Updates
```bash
# Change motion pattern
ros2 param set /velocity_controller motion_pattern "figure8"

# Adjust speeds
ros2 param set /velocity_controller max_linear_speed 0.15
ros2 param set /velocity_controller max_angular_speed 0.8

# Change publish rate
ros2 param set /velocity_controller publish_rate 20.0
```

## Motion Patterns

### Available Patterns
1. **Square**: Moves in a square pattern (forward → turn → forward → turn)
2. **Circle**: Continuous circular motion
3. **Figure-8**: Figure-8 pattern using sinusoidal angular velocity
4. **Stop**: Robot remains stationary

### Pattern Switching
```bash
# Switch to circle pattern
ros2 param set /velocity_controller motion_pattern "circle"

# Switch to figure-8
ros2 param set /velocity_controller motion_pattern "figure8"

# Stop robot
ros2 param set /velocity_controller motion_pattern "stop"
```

## RViz2 Visualization

### Display Configuration
The RViz2 configuration includes:
- **RobotModel**: Shows the URDF robot structure
- **LaserScan**: Displays LiDAR point cloud data
- **TF**: Shows coordinate frame hierarchy
- **Odometry**: Visualizes robot pose and trajectory

### Custom RViz2 Config
```bash
# Load custom configuration
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/turtlebot3_simulation/rviz/turtlebot3_view.rviz
```

## Project Structure

```
turtlebot3_simulation/
├── CMakeLists.txt
├── package.xml
├── urdf/
│   └── turtlebot3_burger.urdf          # Robot model
├── launch/
│   ├── turtlebot3_simulation.launch.py # Complete simulation
│   ├── gazebo_only.launch.py           # Gazebo only
│   └── rviz_only.launch.py             # RViz2 only
├── src/
│   ├── velocity_controller.py          # Motion control node
│   ├── teleop_node.py                  # Teleoperation node
│   ├── service_server.py               # Service demonstrations
│   └── action_server.py                # Action demonstrations
├── rviz/
│   └── turtlebot3_view.rviz            # RViz2 configuration
├── config/
│   └── turtlebot3_params.yaml          # Parameter configuration
└── worlds/
    └── empty_world.world               # Gazebo world
```

## Demonstration Script

### Complete Demo
```bash
#!/bin/bash

# Terminal 1: Start simulation
echo "Starting simulation..."
ros2 launch turtlebot3_simulation turtlebot3_simulation.launch.py &

sleep 5

# Terminal 2: Start velocity controller
echo "Starting velocity controller..."
ros2 run turtlebot3_simulation velocity_controller.py &
sleep 2

# Terminal 3: Start service server
echo "Starting service server..."
ros2 run turtlebot3_simulation service_server.py &
sleep 2

echo "Demo running! Use the following commands:"
echo "1. Change motion pattern: ros2 param set /velocity_controller motion_pattern 'circle'"
echo "2. Adjust speed: ros2 param set /velocity_controller max_linear_speed 0.3"
echo "3. Emergency stop: ros2 service call /emergency_stop std_srvs/SetBool '{data: true}'"
echo "4. Get status: ros2 service call /get_status std_srvs/Trigger"
```

## Assignment Requirements Met

### ✅ ROS 2 Core Elements
- **Nodes**: Multiple autonomous processes for different functions
- **Topics**: Named buses for inter-node communication
- **Messages**: Formatted data exchange (Twist, LaserScan, Odometry)
- **Publishers/Subscribers**: Communication pattern implementation
- **Services**: Synchronous request-response communication
- **Actions**: Asynchronous long-duration tasks
- **Parameters**: External configuration with dynamic updates
- **Packages**: Proper ROS 2 package structure
- **Launch Files**: Automated system startup

### ✅ Motion Control (/cmd_vel)
- **Topic**: `/cmd_vel` with geometry_msgs/Twist messages
- **Linear velocity**: Controls forward/backward motion
- **Angular velocity**: Controls rotation
- **Real-time control**: 10Hz publish rate
- **Smooth transitions**: Acceleration limits

### ✅ Parameter-Based Behavior
- **Dynamic speed adjustment**: max_linear_speed, max_angular_speed
- **Motion pattern selection**: square, circle, figure-8, stop
- **Runtime modification**: No code changes required
- **Service-based configuration**: Set speeds via services

### ✅ RViz2 Visualization
- **RobotModel**: URDF visualization
- **LaserScan**: Real-time sensor data
- **TF**: Coordinate frame hierarchy
- **Odometry**: Position and orientation tracking

### ✅ Services and Actions
- **Services**: emergency_stop, speed configuration, status queries
- **Actions**: navigation, patrol, scanning tasks
- **Real-world examples**: Practical robotics applications

## Troubleshooting

### Common Issues

1. **Gazebo doesn't start**
   ```bash
   # Check Gazebo installation
   gazebo --version
   
   # Install missing packages
   sudo apt install gazebo libgazebo-dev
   ```

2. **Robot doesn't move**
   ```bash
   # Check /cmd_vel topic
   ros2 topic echo /cmd_vel
   
   # Verify differential drive plugin
   ros2 param list | grep gazebo
   ```

3. **LiDAR not showing in RViz2**
   ```bash
   # Check /scan topic
   ros2 topic echo /scan
   
   # Verify laser plugin configuration
   ros2 run rviz2 rviz2 -d turtlebot3_view.rviz
   ```

4. **Build errors**
   ```bash
   # Clean build
   rm -rf build/ install/ log/
   colcon build --packages-select turtlebot3_simulation
   ```

### Performance Tips

1. **Increase Gazebo performance**
   ```bash
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models
   export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(pwd)/worlds
   ```

2. **Reduce CPU usage**
   ```bash
   # Lower update rates
   ros2 param set /velocity_controller publish_rate 5.0
   ```

## Conclusion

This ROS 2 Turtlebot3 simulation project successfully demonstrates all core concepts required for the ECE 2318 Robotics course. The implementation provides a comprehensive foundation for understanding ROS 2 architecture, robot control, sensor integration, and practical robotics applications.

The project showcases:
- **Professional ROS 2 package structure**
- **Complete robot simulation with realistic physics**
- **All ROS 2 communication patterns**
- **Real-world robotics applications**
- **Extensible architecture for future development**

This simulation serves as an excellent platform for learning ROS 2 concepts and developing more advanced robotics behaviors and algorithms.
