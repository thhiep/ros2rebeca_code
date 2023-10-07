#!/usr/bin/bash
clear
#colcon build --packages-select ros2demo
source install/setup.bash
ros2 launch ros2demo multi_robots.launch.py

