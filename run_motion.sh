#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "TurtleSim Motion Started"
ros2 run aisd_motion move
