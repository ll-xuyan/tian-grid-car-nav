#!/bin/bash
# 保存地图到 ~/maps/tian_map
source ~/ros_env.sh
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/tian_map
echo 地图已保存到 ~/maps/
