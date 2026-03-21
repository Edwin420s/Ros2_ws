# ECE2318 Robotics Assignment - ROS 2 Robot Simulation

This repository contains a ROS 2 workspace (`ros2_ws`) for the ECE2318 Robotics Assignment. It implements a complete ROS 2 simulation environment of a custom robot utilizing URDF, Launch Files, Nodes, Topics, and RViz2 visualization, fully containerized using Docker.

---

## 🚀 How to Run the Simulation from Scratch

The following steps detail exactly how to build and launch the robot simulation from start to finish, fulfilling all requirements of the assignment.

### 1. Host Screen Setup (Fedora)
Before launching Docker, you must allow the container to project its graphical interface (RViz2) to your host screen. Open a regular Fedora terminal and run:
```bash
sudo dnf install xhost -y
xhost +local:docker
```
*You should see a message stating "non-network local connections being added to access control list".*

### 2. Enter the Docker Simulation Environment
Start the ROS 2 Humble container and link your workspace folder (`~/ros2_ws`) to the container's `/root/ros2_ws` folder.
```bash
docker run -it --rm \
--net=host \
--env="DISPLAY=$DISPLAY" \
--volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
-v ~/ros2_ws:/root/ros2_ws \
osrf/ros:humble-desktop
```
*(Note: A `docker-compose.yml` and `run.sh` are also available in this repository to automate this process).*

### 3. Build & Source the Workspace (Terminal 1)
Inside the Docker container, clear any old builds and compile your `my_robot` assignment package:
```bash
cd /root/ros2_ws
rm -rf build install log
colcon build --packages-select my_robot
source install/setup.bash
```
Now launch the Robot State Publisher and Joint State Publisher to bring the robot to life:
```bash
ros2 launch my_robot rsp.launch.py
```

### 4. Launch RViz2 (Terminal 2 - The "Eyes")
Open a **new terminal tab** on your real computer, find your running container ID (`docker ps`), and attach to it:
```bash
docker exec -it <your_container_id> bash
```
Once inside, activate your ROS 2 environment and run the visualizer:
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export DISPLAY=:0
rviz2
```

**Inside RViz2:**
1. On the left panel under **Global Options**, change **Fixed Frame** from `map` to `odom` (or `base_link`).
2. Click the **Add** button at the bottom left and select **RobotModel**. Your robot will appear!
3. Click **Add** again and select **TF** to visualize the axes of your sensors and joints.

### 5. Move the Robot (Terminal 3 - The "Brain")
To fulfill the motion programming requirement, open a **third terminal**, enter the container (`docker exec -it <id> bash`), and run your custom publisher node:
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run my_robot walker.py
```
*(The robot will now start driving forward in RViz2, publishing at 0.5 m/s!)*

### 6. Changing Speed Dynamically (Parameters Requirement)
You can change the robot's configuration in real time without stopping the script! Try updating its linear speed to 1.5 m/s:
```bash
ros2 param set /robot_walker linear_speed 1.5
```

---

## 📚 Core ROS 2 Elements Implemented (Report Theory)

### 1. Nodes
Nodes are individual, independent executable processes that perform computation.
- **`robot_state_publisher`**: Reads the URDF file and publishes the 3D transforms (`/tf`) of the robot links to the ROS 2 network.
- **`joint_state_publisher`**: Publishes the state of the robot's joints to the `/joint_states` topic.
- **`robot_walker` (`walker.py`)**: A uniquely created script that acts as a custom Python node. It continuously calculates and publishes velocity commands.
- **`fake_odom` (`fake_odom.py`)**: Translates motion velocity commands into `odom` TF transforms out to RViz2.

### 2. Topics & Messages
Topics are named communication buses over which nodes exchange data using a Publisher/Subscriber model.
- **Motion Control Topic (`/cmd_vel`)**: The specific topic used by the `walker.py` node to send motion commands.
- **Messages (`geometry_msgs/msg/Twist`)**: The exact data structure sent over the `/cmd_vel` topic. It contains linear (`x`, `y`, `z`) and angular (`x`, `y`, `z`) velocity components. In this project, `msg.linear.x` sets the forward speed, and `msg.angular.z` sets the turning speed.

### 3. Publishers & Subscribers
- **Publisher**: The `walker.py` script acts as the Publisher, actively sending velocity data out to the network.
- **Subscriber**: The simulation environment (and `fake_odom.py`) acts as the Subscriber, receiving the data to move the robot dynamically.

### 4. Parameters
Parameters are configurable node settings that can be changed dynamically at runtime without modifying the underlying code.
- **`linear_speed`**: Defined inside `walker.py`, this allows the user or launch file to externalize and adjust the forward velocity of the robot on the fly. 

### 5. Packages and Launch Files
- **Packages**: Directories used to organize software and dependencies (e.g., the `my_robot` folder). They were initialized via `ros2 pkg create --build-type ament_cmake my_robot`.
- **Launch Files**: Scripts (like `rsp.launch.py`) used to automate starting and configuring multiple nodes sequentially rather than manually invoking them one by one.

### 6. Services vs. Actions
*As defined for the assignment report:*
- **Services**: Used for quick, synchronous "Request-Response" tasks (e.g., "turn on the camera" or "read the current sensor value"). The calling node waits until the service completes.
- **Actions**: Used for long-running, asynchronous, goal-oriented tasks that provide continuous feedback (e.g., "navigate to the kitchen", "rotate 360 degrees", "25% complete") and can be canceled mid-execution.

---

## Deliverables Checklist
- [x] **Source Code**: Prepared in `~/ros2_ws/src/my_robot`.
- [x] **Visualization**: Visualized the generic shape and LiDAR sensor using the `RobotModel` and `TF` displays in RViz2.
- [x] **Motion Control**: Implemented using `/cmd_vel` via a publisher node.
- [x] **Parameters**: Configured `linear_speed` dynamically.
- [x] **Execution**: Completely containerized via Docker and automated builds.
