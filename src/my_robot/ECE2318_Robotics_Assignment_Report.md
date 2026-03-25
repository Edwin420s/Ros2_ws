# ECE2318 Robotics Assignment – ROS 2 Simulation Report

**Course**: ECE2318 Robotics
**Project**: ROS 2 Simulation of a Mobile Robot
**Date**: March 25, 2026
**Group Members**: [List 7 Group Members Here]

---

## 1. Introduction to the ROS 2 Architecture & Elements

Robot Operating System 2 (ROS 2) is a flexible and distributed framework for developing robotic applications. In this assignment, a complete ROS 2 workspace was built to simulate a differential-drive mobile robot with an onboard robotic arm and external sensors. The system is entirely distributed, capitalizing on the robust DDS (Data Distribution Service) middleware.

### Explanation of Key ROS 2 Elements

To understand the architecture built, we must define the core elements of ROS 2 utilized in our project:

- **Nodes**: A node is an executable process that performs a specific computation. Our project consists of multiple nodes operating concurrently. For example, `walker.py` computes motion behaviors, `fake_odom.py` simulates the robot's physical movement, and `sensor_simulator.py` generates artificial LiDAR and IMU data. Segregating these tasks ensures modularity.
- **Topics**: Topics provide a named, asynchronous bus over which nodes exchange data using a Publisher-Subscriber model. For example, the `/cmd_vel` topic is used to stream motion commands from `walker.py` to `fake_odom.py`.
- **Messages**: A message is a strictly-typed data structure used when publishing or subscribing to a topic. For motion control, the `geometry_msgs/Twist` message type is used. Our virtual sensors publish messages like `sensor_msgs/LaserScan`. 
- **Publishers**: A publisher is an entity within a node that broadcasts messages onto a specific topic. The `walker.py` acts as a publisher on `/cmd_vel` and `/wheel_velocities`.
- **Subscribers**: A subscriber is an entity that receives messages from a specific topic. The `fake_odom.py` acts as a subscriber to `/cmd_vel`, reading the commands to update the robot's pose.
- **Services**: Services offer a synchronous, request-response communication model. A service is invoked only when necessary and blocks until a response is received. `robot_services.py` offers the `/set_robot_mode` and `/emergency_stop` services for deterministic state interventions.
- **Actions**: Actions are for long-running tasks. They consist of a goal, continuous feedback, and a final result. The `robot_actions.py` node includes a `/navigate_to_pose` action, which accepts a goal position and repeatedly publishes feedback on the remaining distance.
- **Parameters**: Parameters are configuration values associated with a node that can be dynamically updated at runtime. The `walker.py` node exposes parameters like `linear_speed` and `max_angular_speed`, allowing users to tune the robot's handling dynamically.
- **Packages**: A package is the fundamental build and release unit in ROS 2. Our simulation is encapsulated entirely within the `my_robot` package, managing all dependencies via `package.xml` and build rules via `CMakeLists.txt`.
- **Launch Files**: Launch files (`rsp.launch.py`) are Python scripts detailing the declarative state of the runtime environment. They manage starting multiple nodes simultaneously, mapping topics, and loading parameters. Our launch script intelligently boots all nodes (walker, fake_odom, sensor_simulator, robot_state_publisher) with a single command.

---

## 2. Motion Control via ROS 2 Topics

### Topic Used for Motion Control
The primary topic used to control the robot’s continuous physical motion is **`/cmd_vel`**. This topic utilizes the standard `geometry_msgs/msg/Twist` message format, which strictly divides motion into *linear* (meters per second) and *angular* (radians per second) axes. 

In our differential-drive architecture:
- `cmd.linear.x`: Controls the forward and backward translational speed.
- `cmd.angular.z`: Controls the rotational speed around the robot's vertical axis (turning).

### Increasing or Decreasing Velocity Values
When the `linear.x` value published to `/cmd_vel` is **increased**, the robot accelerates to move forward more rapidly. When `linear.x` is made **negative** (decreased below zero), it drives in reverse. 
When `angular.z` is **increased**, the robot rotates counter-clockwise (to the left) faster. If **decreased** into negative values, the robot turns clockwise (to the right). Combining both simulates complex arcs and spiral trajectories. 

In `walker.py`, these values are not changed abruptly. We implemented a smoothing algorithm that organically ramps the velocities incrementally to prevent jerking.

### How Subscribers Receive Commands
When `walker.py` publishes a `Twist` message to `/cmd_vel`, the middleware guarantees delivery to all registered subscribers. The `fake_odom.py` node subscribes to `/cmd_vel`. Within its `cmd_vel_cb` (callback) function, it saves the current linear and angular velocities. Then, running on a rapid 50Hz update timer, the node integrates these velocities over the elapsed time interval ($x += v * \cos(\theta) * dt$, $y += v * \sin(\theta) * dt$) to logically move the 3D model through the simulation grid. It then broadcasts the new coordinates to TF2 (Transform) and publishes an `Odometry` message to let the visualization tools track the new position.

---

## 3. Dynamically Modifying Robot Behavior with Parameters

ROS 2 allows for dynamic adjustments via **Parameters**. In `walker.py`, we defined four key parameters allowing runtime configuration:
1. `linear_speed` (default: 0.55 m/s)
2. `max_angular_speed` (default: 0.45 rad/s)
3. `accel_rate` (default: 0.06 m/s²)
4. `decel_rate` (default: 0.10 m/s²)

**Modifying the Speed**: We can modify these parameters continuously via the CLI to alter the robot behavior. By running:
```bash
ros2 param set /robot_walker linear_speed 0.8
ros2 param set /robot_walker max_angular_speed 1.0
```
This forces the internal state machine of `walker.py` to target the higher speed limit for its Cruise Exploration and Turns, noticeably increasing the simulated robot's momentum without pausing or restarting the software. The robot behavior pivots from a slow, methodical crawl to an aggressive sprint, visualizing how real hardware tuning can be scaled via ROS parameters in the field.

---

## 4. Role of Packages and Launch Files in the Simulation

### Packages
The `my_robot` package ensures modular isolation. It structures our Python scripts (nodes) inside the `/src` directory, and model configurations in the `/urdf` directory. The package guarantees that ROS 2 CLI commands (e.g., `ros2 run my_robot walker.py`) can locate our custom executables easily.

### Launch Files
The launch file (`rsp.launch.py`) acts as the orchestrator of the simulation. Without a launch file, a user would need to open eight individual terminal windows to start each node manually. The launch file automates:
1. Reading the Physical Robot Model (URDF XML).
2. Starting the `robot_state_publisher` to distribute static structural links.
3. Booting up the motion `walker.py` node.
4. Launching the logical position tracker `fake_odom.py`.
5. Firing up the `sensor_simulator.py` to project imaginary LiDAR rays into the environment.
6. Booting up arm operation and pick controller nodes.

It condenses infinite complexity into a single command: `ros2 launch my_robot rsp.launch.py`.

---

## 5. RViz2 Visualization and Sensor Integration

**RViz2** is the premier 3D visualization tool in ROS 2. It bridges the gap between raw terminal data and spatial comprehension. 
Our robot's physical structure, articulated in the URDF file, possesses a base frame with multiple mounted sensors, explicitly defined with physical orientations and positions.

*(Note for Group: Please run `ros2 launch my_robot rsp.launch.py` and then open RViz2 with `rviz2 -d src/my_robot/rviz/urdf_config.rviz` to take the screenshot below)*

**[ INSERT SCREENSHOT OF RVIZ2 HERE SHOWING THE ROBOT MODEL AND LIDAR RED DOTS ]**
*Figure 1: RViz2 interface displaying the mobile robot's URDF model, articulated arm, and the simulated LaserScan data (red points) from the LiDAR sensor reflecting off virtual obstacles.*

### Visualized Sensors:
- **LiDAR Sensor**: Mounted on `lidar_link`. It acts as the robot's eyes, projecting 360 rays per frame. The `sensor_simulator.py` mathematically computes ray-box intersections against virtual obstacles in real-time, outputting `LaserScan` arrays that RViz paints as solid red hit dots in 3D space.
- **Camera Lens**: Mounted on `camera_link`. It captures an RGB simulated array that can be viewed using an Image display in RViz2.

By launching RViz2 with our saved configuration (`urdf_config.rviz`), we instantly see the blue chassis, articulated orange arm, and actively spinning wheels synchronized perfectly with the `walker.py` command outputs.

---

## 6. Utilizing Services and Actions in Robotics

While publish-subscribe is perfect for continuous sensor data, it falls short for procedural tasks.

### Services (Example: Emergency Stop)
Services provide instantaneous, blocking interventions. In this project, `robot_services.py` offers an `/emergency_stop` service. If an operator sees the robot approaching an obstacle, they trigger the service. The service executes an immediate callback, forcing `/cmd_vel` to absolute zeroes and halting movement entirely. It is guaranteed delivery with a success boolean returned to the operator, making it ideal for high-priority physical interventions like an E-Stop.

### Actions (Example: Navigate To Pose)
Actions excel at long-running goals that require extended time and continuous feedback. In our `robot_actions.py`, we implemented a `/navigate_to_pose` feature. An operator sends a singular coordinate point (e.g., [2.0, 3.5]). The action server begins a long loop driving the robot towards the target. While driving, it continuously yields **Feedback** (e.g., "Navigating: 1.2m to goal"). Let's assume an obstacle blocks the path midway; the action can be **Cancelled** safely. If it succeeds, it returns a **Result** indicating the destination was seamlessly reached. This makes Actions the standard for complex robotic navigation and manipulation sequences.
