# 田字格小车仿真 · 操作手册

## 一、完整启动流程（开 3 个终端，快捷键 Ctrl+Alt+T）

### 终端 1：启动仿真（地图 + 小车）
    ~/start_car.sh
等待 30~60 秒：Gazebo 窗口弹出、小车出现在场地中央。
（若小车没出现：Ctrl+C 停掉，重新运行一次即可——偶尔生成超时，重启就好）

### 终端 2：启动 SLAM 建图 + RViz
    ~/start_slam.sh
等待约 20 秒 RViz 自动打开（已配好地图/激光/机器人显示）。

### 终端 3：键盘遥控开车
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
鼠标点进这个终端（保持焦点！），按键盘操作：

    i = 前进        , = 后退
    j = 左转        l = 右转        k = 停车
    q = 加速        z = 减速
    （扫图建议先连按几次 z，把速度降到 0.2~0.3 再开）

## 二、扫图（手动驾驶建图）
- 慢速绕外圈走廊跑一整圈
- 东西向、南北向中央走廊各走一趟
- 看着 RViz 里的地图逐渐生长；轻微错位正常（SLAM 自动校正）

## 三、保存地图
    ~/save_map.sh
（或在 RViz 左侧 SlamToolboxPlugin 面板点 Save Map 按钮）
保存位置：~/maps/tian_map.pgm + tian_map.yaml

## 四、关闭（一键停止所有程序）
任意终端执行（推荐）：
    ~/stop_sim.sh
会一键关闭：Gazebo 仿真、小车、SLAM、导航节点、RViz、键盘遥控 等全部相关程序。
（也可以在每个终端里分别按 Ctrl+C 停）

> 想连整个虚拟机一起关机：`sudo shutdown -h now`（或 VirtualBox 窗口直接关闭选"关机"）

## 五、脚本一览
| 命令 | 作用 |
|---|---|
| ~/start_car.sh    | 启动地图 + 小车（自动清理旧进程） |
| ~/start_slam.sh   | 启动 SLAM 建图 + RViz |
| ~/start_square.sh | 只启动地图（不含小车） |
| ~/save_map.sh     | 保存当前地图 |
| ~/stop_sim.sh     | 一键停止所有程序（仿真/导航/RViz/遥控） |
| ~/start_pp_nav.sh | **自主导航（方案B）+ RViz 导航界面** |
| ~/pp_point.sh     | 命令行发送 起点/终点（不用 RViz 时） |

## 六、自主导航 · 方案 B（Pure Pursuit 循迹）【已实测通过】

思路：路径 = 走廊的"车道中心线"网络（田字的 # 骨架）；行驶中用
Pure Pursuit（前视追踪）+ PID（横向纠偏）每个控制周期闭环修正一次，
小车始终贴着车道正中心行驶。

### 用法（开 2 个终端）
终端 1：启动仿真
    ~/start_car.sh

终端 2：启动导航 + RViz 导航界面
    ~/start_pp_nav.sh

然后在 RViz 里选点：
    1) 先点工具栏 [2D Pose Estimate]，在小车当前位置点一下（选起点，可选）
    2) 再点 [2D Goal Pose]，点想要的终点 → 小车自动出发，到点自动停车

提示：
- 想从指定的起点出发？先把小车开过去（或用遥控放好），再点起点→终点
- 点选的坐标会吸附到最近的车道中心线上（保证走中线）
- 若目标在车身后方（掉头），小车会先原地转向对准、再行驶

### 设定速度（运行中随时可调）
    ros2 param set /pure_pursuit_nav speed 0.40      # 调整为 0.4 m/s

### 不用 RViz 的选点方式（命令行；导航节点需已启动）
    ~/pp_point.sh start -4.25 -4.25     # 起点（可选）
    ~/pp_point.sh goal   4.25  4.25     # 终点 → 立即出发

### 常用参数（可运行中调整）
| 参数 | 默认 | 作用 |
|---|---|---|
| speed | 0.25 | 巡航速度 m/s |
| kp_lateral | 0.8 | PID 横向纠偏强度 |
| lookahead_min / max | 0.35 / 0.60 | 前视距离范围 m |
| corner_cut | 0.18 | 拐角切角长度 m |
| spin_alpha_threshold | 2.0 | 目标在身后多少弧度时先原地转向 |
| regulated | false | true = 弯道自动减速 |
| goal_tolerance | 0.15 | 到达判定半径 m |

    查看参数： ros2 param list /pure_pursuit_nav
    修改示例： ros2 param set /pure_pursuit_nav regulated true

### 运行记录
每次任务的轨迹自动存到 ~/pp_log/（pp_run_日期_时间.csv，可用 Excel 打开；
列：时间/坐标/速度/角速度/横向误差/剩余距离）。

## 七、注意事项
1. 启动一律用脚本（会自动清理旧进程，防止端口冲突）
2. 顺序：先 start_car，再 start_slam（或再 start_pp_nav）
3. 重启仿真时 SLAM / 导航要一起重启（否则与仿真失联）
4. 示例环境：Ubuntu 22.04 + ROS2 Humble（账号 ros，仅本地虚拟机演示用）
5. 文件位置：
   - 地图世界：~/gazebo_ws/src/my_gazebo/worlds/square_10m.world
   - 小车模型：~/gazebo_ws/src/my_gazebo/urdf/my_car.urdf
   - SLAM 参数：~/gazebo_ws/src/my_gazebo/config/slam_params.yaml
   - SLAM 界面：~/gazebo_ws/src/my_gazebo/config/slam_view.rviz
   - 导航节点：~/gazebo_ws/src/my_gazebo/scripts/pure_pursuit_nav.py
   - 导航界面：~/gazebo_ws/src/my_gazebo/config/nav_view.rviz
   （改世界/URDF/脚本文件免重新编译，直接重启脚本即可生效）
