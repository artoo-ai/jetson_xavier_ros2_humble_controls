#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/install/setup.bash" --
#source "/root/ros2_ws/install/local_setup.bash" --
#source "/root/ros2_ws/install/setup.bash" --

source "/opt/ros/$ROS_DISTRO/install/setup.bash" --
source "/root/ros2_ws/install/local_setup.bash" --
source "/root/ros2_ws/install/setup.bash" --

# Welcome information
echo "Teleop Twist Controller ROS2 Docker Image"
echo "---------------------"
echo "ROS_DISTRO $ROS_DISTRO"
echo "ROS_ROOT   $ROS_ROOT"
echo 'DDS middleware: ' $RMW_IMPLEMENTATION 
echo "---------------------"

ros2 launch pwm_pca9685 artoo_diff_drive.launch.py