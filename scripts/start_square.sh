#!/bin/bash
# 只启动田字格地图
source ~/ros_env.sh
~/clean_gazebo.sh
source /opt/ros/humble/setup.bash
exec ros2 launch my_gazebo square_10m.launch.py
