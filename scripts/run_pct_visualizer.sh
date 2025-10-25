#!/bin/bash
# Wrapper script to run PCT visualizer with conda environment

# Activate conda environment
source ~/miniconda3/etc/profile.d/conda.sh
conda activate dimos-ros

# Source ROS 2 workspace
source /opt/ros/jazzy/setup.bash
source ~/autonomy_stack_mecanum_wheel_platform/install/setup.bash

# Run the launch file with any arguments passed to this script
ros2 launch pct_planner pct_visualizer.launch.py "$@"
