# ROS2 Mobile Robot Simulation

A professional ROS2 simulation package featuring a 4-wheel mobile robot with a 4-DOF robotic arm, advanced sensors, and autonomous navigation capabilities.

## Overview

This project implements a complete robotic system with:
- **Mobile Platform**: 4-wheel differential drive robot
- **Manipulator**: 4-DOF robotic arm with inverse kinematics
- **Sensors**: LiDAR, IMU, and Camera simulation
- **Autonomy**: Pick-and-place functionality with obstacle avoidance
- **Services & Actions**: Full ROS2 communication interface

## Quick Start

### Prerequisites
- ROS2 Humble
- Docker (optional, for containerized deployment)
- Python 3.10+

### Installation

```bash
# Clone and build
cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash

# Launch simulation
ros2 launch my_robot rsp.launch.py
```

### Docker Deployment

```bash
# Launch with GUI support
xhost +local:docker
bash run.sh
```

## Architecture

### Core Components

| Component | Description |
|-----------|-------------|
| **Walker Node** | 4-state motion machine (EXPLORE/TURN/SPIRAL/PAUSE) |
| **Odometry Node** | Dead-reckoning with visual wheel feedback |
| **Sensor Simulator** | Ray-cast LiDAR, IMU with noise, object detection |
| **Arm Controller** | IK-based pick-and-place with smooth trajectories |
| **Navigation** | Obstacle-aware approach to detected objects |
| **Services** | Mode control, status queries, emergency stop |
| **Actions** | Long-running tasks with feedback |

### Key Topics

- `/cmd_vel` - Motion control (geometry_msgs/Twist)
- `/odom` - Robot odometry (nav_msgs/Odometry)
- `/scan` - 360° LiDAR data (sensor_msgs/LaserScan)
- `/imu` - IMU readings (sensor_msgs/Imu)
- `/joint_states` - All joint positions (sensor_msgs/JointState)
- `/detected_object` - Object detection (geometry_msgs/Point)

## Features

### Motion Control
- Smooth velocity ramping for realistic movement
- Dynamic parameter adjustment:
```bash
ros2 param set /robot_walker linear_speed 1.0
```

### Services
- `/set_robot_mode` - Toggle manual/autonomous operation
- `/get_robot_status` - System status query
- `/trigger_pick` - Manual pick sequence
- `/emergency_stop` - Immediate motion halt

### Actions
- `/navigate_to_pose` - Point-to-point navigation
- `/pick_and_place` - Complete manipulation sequence
- `/explore_area` - Systematic area coverage

## Robot Configuration

### Physical Specifications
- **Chassis**: 3×2×0.9m with inertial properties
- **Wheels**: 4× continuous joints with dynamics
- **Arm**: 4-DOF (base yaw, shoulder, elbow, wrist)
- **Gripper**: 2-finger prismatic mechanism
- **Sensors**: LiDAR (360°), Camera, IMU

### URDF Features
- Complete collision geometry
- Accurate inertial properties
- Joint limits and dynamics
- Visual mesh representations

## Development

### Package Structure
```
my_robot/
├── src/           # Python nodes
├── launch/        # Launch configurations
├── urdf/          # Robot model
├── rviz/          # Visualization configs
├── config/        # Parameter files
└── CMakeLists.txt # Build configuration
```

### Adding New Nodes
1. Create Python script in `src/`
2. Add to `CMakeLists.txt` install section
3. Update `package.xml` if new dependencies
4. Include in launch file as needed

## Testing

### Unit Tests
```bash
colcon test --packages-select my_robot
```

### Integration Tests
```bash
# Test all services
ros2 run my_robot demo_services_actions.py

# Monitor system
ros2 topic echo /robot_status
```

## Troubleshooting

### Common Issues
- **Build failures**: Ensure all dependencies in `package.xml`
- **TF errors**: Wait 2-3 seconds after launch for stabilization
- **Service unavailable**: Check node startup with `ros2 node list`

### Debug Commands
```bash
# System overview
ros2 node list
ros2 topic list
ros2 service list

# Real-time monitoring
ros2 topic hz /scan
ros2 topic echo /cmd_vel
```

## License

Apache License 2.0

## Contributing

1. Fork the repository
2. Create feature branch
3. Make changes with tests
4. Submit pull request  
