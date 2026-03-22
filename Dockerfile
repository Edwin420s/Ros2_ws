FROM osrf/ros:humble-desktop

# Install joint_state_publisher (missing in the base image)
RUN apt-get update -o Acquire::ForceIPv4=true && apt-get install -y -o Acquire::ForceIPv4=true \
    ros-humble-joint-state-publisher ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# Add source command to bashrc so we don't need to do it every time
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /root/ros2_ws
