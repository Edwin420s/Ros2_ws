# ECE2318 Robotics Assignment — ROS 2 Robot Simulation (v2)

A complete ROS 2 simulation of a custom 4-wheel robot with a 4-DOF robotic arm (IK-controlled), working LiDAR + IMU sensors, and autonomous pick-and-place behaviour. Fully containerised with Docker.

---

## 🚀 Quick Start

```bash
# 1. Allow Docker to display GUI windows on your Fedora host
sudo dnf install xorg-x11-server-utils -y
xhost +local:docker

# 2. Launch everything (build + all nodes + RViz2 + Gazebo)
bash run.sh
```
```

./run.sh
```
That's it. `docker compose up --build` starts:

| Service    | What it does |
|------------|--------------|
| `ros_base` | Builds workspace, then launches ALL ROS nodes via `rsp.launch.py` |
| `rviz2`    | Opens RViz2 with LiDAR scan, TF, odometry trail, and robot model |
| `gazebo`   | Starts Gazebo simulation |

---

## 🧠 Architecture — Nodes & Topics

```
walker.py       →  /cmd_vel  →  fake_odom.py  →  /odom  →  sensor_simulator.py
                               /wheel_velocities          →  /detected_object
                               /joint_states (wheels)     →  arm_controller.py
                                                          →  /joint_states (arm)
pick_controller.py ← /scan, /odom, /detected_object
pick_controller.py → /cmd_vel
```

### Nodes

| Node | Description |
|------|-------------|
| `robot_state_publisher` | Publishes robot TF tree from URDF |
| `fake_odom` | 50 Hz dead-reckoning odometry + visual wheel spin |
| `walker` | 4-state motion machine (EXPLORE/TURN/SPIRAL/PAUSE) with smooth velocity ramping |
| `sensor_simulator` | Ray-cast LiDAR, IMU with noise + covariance, object detection |
| `arm_controller` | 8-state smooth IK pick-and-place (non-blocking, 20 Hz) |
| `pick_controller` | Navigation to detected objects with obstacle avoidance |

---

## 🤖 Robot Design

- **Base**: 3×2×0.9 m chassis, full `<inertial>` and `<collision>` tags
- **4 Wheels**: `continuous` joints with `<dynamics damping/friction>` — spin visually
- **Sensors**:
  - LiDAR cylinder on front-top
  - Camera box + lens cylinder on front face
  - IMU chip on top of chassis
- **Arm**: 4-DOF (base yaw + shoulder + elbow + wrist), all `revolute` with limits
- **Gripper**: 2 prismatic fingers + yellow fingertip spheres

---

## 🎮 Motion Behaviour (`walker.py`)

Walker runs a 4-state machine at 20 Hz:

1. **EXPLORE** — forward drive with gentle sinusoidal steering sway
2. **TURN** — banked turn (alternating left/right each cycle)
3. **SPIRAL** — widening arc to sweep new area
4. **PAUSE** — smooth deceleration stop before next state

Velocity is **ramped** (no instant jumps) — feels like a real vehicle.

### Change speed dynamically
```bash
ros2 param set /robot_walker linear_speed 1.0
```

---

## 🦾 Arm Pick-and-Place (`arm_controller.py`)

8-state cycle (runs forever):

`HOME → OPEN_GRIPPER → REACH_APPROACH → REACH_OBJECT → CLOSE_GRIPPER → LIFT → DEPOSIT → HOME`

- Uses 2-link planar **inverse kinematics** (law of cosines)
- Joint angles **interpolated smoothly** at 20 Hz — no blocking sleep()
- Falls back to internal demo objects when no `/detected_object` received

---

## 📡 Sensors (`sensor_simulator.py`)

| Topic | Type | Details |
|-------|------|---------|
| `/scan` | `LaserScan` | 360°, 1° resolution, ray-cast against obstacles + walls |
| `/imu` | `Imu` | Acceleration, angular velocity, orientation + Gaussian noise + covariance |
| `/detected_object` | `Point` | Published when robot is within 2.5 m of an object |

---

## 🧭 Navigation (`pick_controller.py`)

5-state nav machine:

1. **SEARCHING** — spin to scan
2. **APPROACHING** — proportional P-control to drive toward target
3. **WAITING** — hand off to arm (6 s)
4. **RETURNING** — drive back to origin
5. **IDLE** → back to SEARCHING

Obstacle avoidance via ±25° front LiDAR sector check.

---

## 📋 Deliverables Checklist

- [x] URDF with `<inertial>`, `<collision>`, `<dynamics>`, `<limit>` on all joints  
- [x] LiDAR, Camera, IMU sensor links  
- [x] Robotic arm — 4-DOF + 2-finger prismatic gripper  
- [x] Smooth motion with velocity ramping and state machine  
- [x] Wheel joints visually spinning in RViz  
- [x] Working LiDAR scan (`/scan`) visible in RViz as orange spheres  
- [x] IMU data published with covariance  
- [x] Autonomous pick-and-place with IK  
- [x] Obstacle avoidance during navigation  
- [x] Dynamic parameter `linear_speed`  
- [x] Fully containerised with Docker Compose  
