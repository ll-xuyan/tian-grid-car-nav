#!/bin/bash
# =============================================================
# 用命令行发送 起点/终点（不用 RViz 时）
# 用法:
#   ~/pp_point.sh start X Y [YAW度]    # 发送起点（可选）
#   ~/pp_point.sh goal  X Y [YAW度]    # 发送终点（立即出发）
# 示例:
#   ~/pp_point.sh goal 4.25 4.25       # 开到右上角路口
#   ~/pp_point.sh start -4.25 -4.25    # 先把起点设在左下角
# =============================================================
source ~/ros_env.sh

mode=$1
x=$2
y=$3
yaw_deg=${4:-0}

if [ -z "$mode" ] || [ -z "$x" ] || [ -z "$y" ]; then
    echo "用法: $0 {start|goal} X Y [YAW度]"
    exit 1
fi

read z w < <(python3 -c "
import math
y = math.radians($yaw_deg)
print(f'{math.sin(y/2):.6f} {math.cos(y/2):.6f}')")

if [ "$mode" = "start" ]; then
    ros2 topic pub --once /initialpose \
      geometry_msgs/msg/PoseWithCovarianceStamped \
      "{header: {frame_id: 'odom'}, pose: {pose: {position: {x: $x, y: $y}, orientation: {z: $z, w: $w}}}}"
elif [ "$mode" = "goal" ]; then
    ros2 topic pub --once /goal_pose \
      geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: 'odom'}, pose: {position: {x: $x, y: $y}, orientation: {z: $z, w: $w}}}"
else
    echo "第一个参数必须是 start 或 goal"
    exit 1
fi
