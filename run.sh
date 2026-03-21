#!/bin/bash

# Ensure X11 forwarding is configured
echo "Configuring X11 for Docker..."
xhost +local:docker || echo "Warning: xhost command failed. GUI might not work."

# Bring up the Docker Compose cluster
echo "Starting ROS2 Simulation..."
docker compose up --build
