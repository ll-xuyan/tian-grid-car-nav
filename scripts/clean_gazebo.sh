#!/bin/bash
# 彻底清理所有仿真/导航相关进程（先软后硬）
pkill -x gzserver 2>/dev/null
pkill -x gzclient 2>/dev/null
pkill -f tb3_simulation 2>/dev/null
pkill -f my_gazebo 2>/dev/null              # 仿真世界 + launch + 导航节点
pkill -f pure_pursuit_nav 2>/dev/null       # 导航节点（双保险）
pkill -f teleop_twist_keyboard 2>/dev/null  # 键盘遥控
pkill -f nav2_container 2>/dev/null
pkill -f spawn_entity 2>/dev/null
pkill -f robot_state_publisher 2>/dev/null
pkill -f rviz2 2>/dev/null                  # RViz 窗口
sleep 2
pkill -9 -x gzserver 2>/dev/null
pkill -9 -x gzclient 2>/dev/null
pkill -9 -f teleop_twist_keyboard 2>/dev/null
sleep 1
