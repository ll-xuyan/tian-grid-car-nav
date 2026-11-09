#!/bin/bash
# 统一环境配置（所有启动脚本引用）
source /opt/ros/humble/setup.bash
source ~/gazebo_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:$HOME/.gazebo/models:/opt/ros/humble/share/turtlebot3_gazebo/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export TURTLEBOT3_MODEL=waffle
