#!/bin/bash
# 启动田字格地图 + 四轮小车
source ~/ros_env.sh
~/clean_gazebo.sh
echo ==========================================
echo 启动田字格地图 + 小车，等约30秒
echo 停止: Ctrl+C     方向盘: 另开终端 ros2 run teleop_twist_keyboard teleop_twist_keyboard
echo ==========================================
exec ros2 launch my_gazebo my_car.launch.py
