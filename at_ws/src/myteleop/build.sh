#!/bin/bash

set -e

if [ -z $ROS_DISTRO ]; then
    source ~/Projects/ros2_ws/ros2_setup.sh
fi

cd ../../

colcon build --symlink-install --packages-up-to myteleop
