#!/bin/bash

# Source the ROS 2 environment (adjust if using a different ROS 2 version)
# source /opt/ros/jazzy/setup.bash

export ROS_PYTHON_EXECUTABLE=/home/shirox/workspace/robot/.venv/bin/python3

# rm -rf build install log

# Build the workspace
colcon build --cmake-args -DPython3_FIND_VIRTUALENV="ONLY"

# Source the workspace
source install/setup.bash
