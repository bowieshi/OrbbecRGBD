#!/bin/zsh
cd /home/bowie/Desktop/scanAPP/data/scene
source /opt/ros/humble/setup.zsh
source /home/bowie/Desktop/scanAPP/camera_ws/install/setup.zsh
ros2 launch /home/bowie/Desktop/scanAPP/camera_ws/launch/camera_launch.py