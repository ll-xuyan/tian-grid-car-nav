#!/bin/bash
# =============================================================
# 自主导航（方案B：Pure Pursuit 循迹）— 操作终端
# 前提：先在另一个终端运行 ~/start_car.sh 启动仿真
# 使用：~/start_pp_nav.sh
#   然后在本窗口看状态；RViz 里：
#     1) [2D Pose Estimate] 点小车当前位置（起点，可选）
#     2) [2D Goal Pose]    点想要的终点 → 小车自动出发
# =============================================================
source ~/ros_env.sh

if ! ros2 topic list 2>/dev/null | grep -q '^/odom$'; then
    echo "⚠️  未检测到 /odom 话题 —— 仿真还没启动。"
    echo "   请先开另一个终端运行:  ~/start_car.sh"
    exit 1
fi

echo "=============================================="
echo " Pure Pursuit 导航启动中（含 RViz 导航界面）"
echo " 选点: 先 [2D Pose Estimate] 起点(可选)"
echo "       再 [2D Goal Pose]   终点"
echo " 调速度: ros2 param set /pure_pursuit_nav speed 0.35"
echo " 停止: Ctrl+C"
echo "=============================================="
exec ros2 launch my_gazebo nav_pp.launch.py
