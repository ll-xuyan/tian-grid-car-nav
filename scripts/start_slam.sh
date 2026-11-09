#!/bin/bash
# 启动 SLAM 建图 + RViz 需先运行 start_car.sh
source ~/ros_env.sh
echo ==========================================
echo SLAM 建图中, RViz 打开后驾驶扫描
echo 停止: Ctrl+C     保存地图: ~/save_map.sh
echo ==========================================
exec ros2 launch my_gazebo slam.launch.py
