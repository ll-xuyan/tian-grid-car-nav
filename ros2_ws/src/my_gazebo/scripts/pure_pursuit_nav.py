#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
# pure_pursuit_nav.py — 田字格小车 · 自主导航（方案 B：自己写的算法）
#
# 三个需求 → 对应实现：
#   1. 车道居中 : 路径 = 走廊中心线网络（3横3纵），Pure Pursuit 循迹 +
#                  PID 横向纠偏，每个控制周期闭环修正横向误差
#   2. 选起终点 : RViz 里 "2D Pose Estimate" 点起点（可选）、"2D Goal Pose"
#                  点终点；也可用参数或命令行话题方式（见 pp_point.sh）
#   3. 设定速度 : 参数 speed（运行中可 ros2 param set 实时调整），可选弯道减速
#
# 运行前提：仿真已启动（~/start_car.sh），本节点需要 /odom、/scan，输出 /cmd_vel
# 推荐用法：ros2 launch my_gazebo nav_pp.launch.py   （同时打开 RViz 导航界面）
# =============================================================================

import bisect
import csv
import heapq
import itertools
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# ----------------------- 场地常量（与 square_10m.world 一致） -----------------------
# 田字格：10m x 10m 外墙，四个 2.75m 方块（中心 ±2.125），走廊宽 1.5m。
# 可行驶走廊的中心线是 3 横 3 纵共 6 条线（田字的 "#" 骨架），坐标如下：
CORRIDOR_COORDS = (-4.25, 0.0, 4.25)   # 中心线的 x / y 坐标
GRID_LIMIT = 4.25                       # 中心线延伸范围（±4.25）


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def dist2d(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def snap_to_network(x, y):
    """把任意点吸附到最近的车道中心线上。
    返回 (sx, sy, kind, coord)：吸附点坐标、所在线类型('h'/'v')、线坐标。"""
    best = None
    for yl in CORRIDOR_COORDS:          # 横线 y = yl
        px = min(max(x, -GRID_LIMIT), GRID_LIMIT)
        d = math.hypot(x - px, y - yl)
        if best is None or d < best[0]:
            best = (d, px, yl, 'h', yl)
    for xl in CORRIDOR_COORDS:          # 竖线 x = xl
        py = min(max(y, -GRID_LIMIT), GRID_LIMIT)
        d = math.hypot(x - xl, y - py)
        if best is None or d < best[0]:
            best = (d, xl, py, 'v', xl)
    _, sx, sy, kind, coord = best
    return sx, sy, kind, coord


def network_route(a_xy, b_xy):
    """在走廊中心线网络上规划 a→b 的最短路（Dijkstra）。
    返回折线点列表 [(x,y), ...]（均已吸附到中心线）；失败返回 None。"""
    ax, ay, ka, ca = snap_to_network(*a_xy)
    bx, by, kb, cb = snap_to_network(*b_xy)

    # 图节点：9 个交叉点 + 起点 'a' + 终点 'b'
    nodes = {'a': (ax, ay), 'b': (bx, by)}
    for i, cx in enumerate(CORRIDOR_COORDS):
        for j, cy in enumerate(CORRIDOR_COORDS):
            nodes[('j', i, j)] = (cx, cy)

    adj = {k: [] for k in nodes}

    def add_edge(u, v):
        w = dist2d(nodes[u], nodes[v])
        adj[u].append((v, w))
        adj[v].append((u, w))

    # 相邻交叉点之间的走廊段
    n = len(CORRIDOR_COORDS)
    for i in range(n):
        for j in range(n):
            if i + 1 < n:
                add_edge(('j', i, j), ('j', i + 1, j))
            if j + 1 < n:
                add_edge(('j', i, j), ('j', i, j + 1))

    def attach(tag, kind, coord, along):
        """把起点/终点接到其所在中心线上的相邻交叉点"""
        idx = CORRIDOR_COORDS.index(coord)
        cand = set()
        left = [k for k in range(n) if CORRIDOR_COORDS[k] <= along + 1e-6]
        right = [k for k in range(n) if CORRIDOR_COORDS[k] >= along - 1e-6]
        if left:
            cand.add(left[-1])
        if right:
            cand.add(right[0])
        for k in cand:
            jn = ('j', k, idx) if kind == 'h' else ('j', idx, k)
            add_edge(tag, jn)

    attach('a', ka, ca, ax if ka == 'h' else ay)
    attach('b', kb, cb, bx if kb == 'h' else by)
    if ka == kb and abs(ca - cb) < 1e-9:     # 同一条线：允许直接相连
        add_edge('a', 'b')

    # Dijkstra（堆条目带递增计数器，避免距离相等时比较节点名报 TypeError）
    dist = {k: math.inf for k in nodes}
    prev = {}
    dist['a'] = 0.0
    tie = itertools.count()
    pq = [(0.0, next(tie), 'a')]
    while pq:
        d, _, u = heapq.heappop(pq)
        if d > dist[u] + 1e-12:
            continue
        for v, w in adj[u]:
            nd = d + w
            if nd < dist[v] - 1e-9:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, next(tie), v))
    if math.isinf(dist['b']):
        return None
    seq = ['b']
    u = 'b'
    while u != 'a':
        u = prev[u]
        seq.append(u)
    seq.reverse()
    return [nodes[k] for k in seq]


def chamfer_corners(pts, cut):
    """拐角切小斜边：减小 Pure Pursuit 的切角幅度，更贴中线通过路口"""
    if len(pts) < 3 or cut <= 0.0:
        return pts
    out = [pts[0]]
    for i in range(1, len(pts) - 1):
        p = pts[i]
        a = out[-1]
        b = pts[i + 1]
        v1 = (p[0] - a[0], p[1] - a[1])
        v2 = (b[0] - p[0], b[1] - p[1])
        l1 = math.hypot(*v1)
        l2 = math.hypot(*v2)
        if l1 < 1e-6 or l2 < 1e-6:
            continue
        u1 = (v1[0] / l1, v1[1] / l1)
        u2 = (v2[0] / l2, v2[1] / l2)
        dot = max(-1.0, min(1.0, u1[0] * u2[0] + u1[1] * u2[1]))
        if math.acos(dot) < math.radians(12.0):   # 近似直行，不动
            out.append(p)
            continue
        d = min(cut, l1 * 0.5, l2 * 0.5)
        out.append((p[0] - u1[0] * d, p[1] - u1[1] * d))
        out.append((p[0] + u2[0] * d, p[1] + u2[1] * d))
    out.append(pts[-1])
    return out


def dedup(pts, eps=1e-4):
    out = []
    for p in pts:
        if not out or dist2d(p, out[-1]) > eps:
            out.append(p)
    return out


# =============================================================================
# 导航节点
# =============================================================================
class PurePursuitNav(Node):

    def __init__(self):
        super().__init__('pure_pursuit_nav')

        # ---------------- 参数（运行中可用 ros2 param set 实时调整） ----------------
        self.declare_parameter('speed', 0.25)                  # 巡航速度 m/s
        self.declare_parameter('lookahead_min', 0.35)          # 前视距离下限 m
        self.declare_parameter('lookahead_max', 0.60)          # 前视距离上限 m
        self.declare_parameter('lookahead_gain', 1.6)          # L = gain * v
        self.declare_parameter('max_omega', 1.4)               # 最大角速度 rad/s
        self.declare_parameter('goal_tolerance', 0.15)         # 到达判定半径 m
        self.declare_parameter('kp_lateral', 0.8)              # 横向纠偏 P
        self.declare_parameter('ki_lateral', 0.0)              # 横向纠偏 I
        self.declare_parameter('kd_lateral', 0.10)             # 横向纠偏 D
        self.declare_parameter('lateral_correction_max', 0.6)  # 纠偏角速度限幅
        self.declare_parameter('corner_cut', 0.18)             # 路径拐角切角长度 m
        self.declare_parameter('spin_alpha_threshold', 2.0)    # 目标在身后(弧度)时先原地转向
        self.declare_parameter('spin_rate', 1.0)               # 原地转向角速度 rad/s
        self.declare_parameter('regulated', False)             # True=弯道自动减速
        self.declare_parameter('use_scan_guard', True)         # 雷达安全保护
        self.declare_parameter('scan_slow_dist', 0.50)         # 前方该距离内减速
        self.declare_parameter('scan_stop_dist', 0.20)         # 前方该距离内停车
        self.declare_parameter('front_sector_deg', 35.0)       # 前向扇区角度
        self.declare_parameter('control_rate', 20.0)           # 控制频率 Hz
        self.declare_parameter('csv_log_dir', '~/pp_log')      # 运行记录目录
        # 参数模式（不用 RViz 时直接给起终点坐标，NaN = 不用）
        self.declare_parameter('start_x', float('nan'))
        self.declare_parameter('start_y', float('nan'))
        self.declare_parameter('goal_x', float('nan'))
        self.declare_parameter('goal_y', float('nan'))

        # ---------------- 状态 ----------------
        self._car = None            # (x, y, yaw)
        self._have_odom = False
        self._scan = None
        self._scan_t = 0.0
        self._start_raw = None      # 用户点选的起点（原始坐标）
        self._goal_raw = None       # 用户点选的终点（原始坐标）
        self._pts = None            # 当前路径（中心线折线）
        self._cum = []              # 路径累计弧长
        self._total_len = 0.0
        self._seg_idx = 0           # 最近段索引（防回退）
        self._state = 'IDLE'        # IDLE / DRIVING / ARRIVED
        self._pid_e_prev = 0.0
        self._pid_ie = 0.0
        self._t_prev = None
        self._t_start = 0.0
        self._t_last_status = 0.0
        self._t_path_pub = 0.0
        self._stats_max_e = 0.0
        self._stats_len = 0.0
        self._last_car = None
        self._csv_f = None
        self._csv_w = None

        # ---------------- 订阅 / 发布 ----------------
        self.create_subscription(Odometry, '/odom', self._odom_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self._start_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self._pub_path = self.create_publisher(Path, '/pp_path', 10)
        self._pub_status = self.create_publisher(String, '/pp_status', 10)

        rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._control)

        self.get_logger().info(
            'Pure Pursuit 导航已启动 ✓\n'
            '  · 车道居中: 沿走廊中心线行驶 + PID 横向纠偏\n'
            '  · 选点方式: RViz 里先点 [2D Pose Estimate] 选起点(可选)，'
            '再点 [2D Goal Pose] 选终点\n'
            '  · 也可命令行: ros2 topic pub /goal_pose ... 或 ~/pp_point.sh\n'
            '  · 速度调整: ros2 param set /pure_pursuit_nav speed 0.4')

    # ----------------------------- 回调 -----------------------------
    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._car = (p.x, p.y, yaw)
        if not self._have_odom:
            self._have_odom = True
            self.get_logger().info(
                f'收到里程计 ✓ 小车当前位姿 ({p.x:.2f}, {p.y:.2f})，等待任务…')
            self._try_param_mission()

    def _scan_cb(self, msg):
        self._scan = msg
        self._scan_t = time.time()

    def _start_cb(self, msg):
        p = msg.pose.pose.position
        self._start_raw = (p.x, p.y)
        self.get_logger().info(f'🚩 收到起点 ({p.x:.2f}, {p.y:.2f})')
        if self._goal_raw is not None and self._have_odom:
            self._start_mission()
        else:
            self.get_logger().info('   已记录，请接着点 [2D Goal Pose] 选择终点')

    def _goal_cb(self, msg):
        p = msg.pose.position
        self._goal_raw = (p.x, p.y)
        self.get_logger().info(f'🎯 收到终点 ({p.x:.2f}, {p.y:.2f})')
        if self._have_odom:
            self._start_mission()

    # ----------------------------- 任务规划 -----------------------------
    def _try_param_mission(self):
        """参数模式：启动时直接给定了 goal_x/goal_y 就立即出发（测试用）"""
        gx = float(self.get_parameter('goal_x').value)
        gy = float(self.get_parameter('goal_y').value)
        sx = float(self.get_parameter('start_x').value)
        sy = float(self.get_parameter('start_y').value)
        if math.isfinite(sx) and math.isfinite(sy):
            self._start_raw = (sx, sy)
        if math.isfinite(gx) and math.isfinite(gy):
            self._goal_raw = (gx, gy)
            self.get_logger().info(f'参数任务: 起点={self._start_raw} 终点={(gx, gy)}')
            self._start_mission()

    def _start_mission(self):
        if self._car is None or self._goal_raw is None:
            return
        car = (self._car[0], self._car[1])

        # 1) 路由：当前位姿 → (可选起点) → 终点（全部在中心线网络上）
        if self._start_raw is not None:
            s_snap = snap_to_network(*self._start_raw)[:2]
            if dist2d(car, s_snap) > 0.6:
                # 小车不在起点附近：先沿中心线开到起点，再去终点
                leg1 = network_route(car, self._start_raw)
                leg2 = network_route(self._start_raw, self._goal_raw)
                pts = dedup((leg1 or []) + (leg2 or []))
            else:
                pts = network_route(car, self._goal_raw)
        else:
            pts = network_route(car, self._goal_raw)

        if not pts:
            self.get_logger().warn('路径规划失败：点选位置无效')
            return

        # 2) 拐角切角 → 更贴中线
        pts = dedup(chamfer_corners(pts,
                                    float(self.get_parameter('corner_cut').value)))
        if len(pts) < 2:
            self.get_logger().info('导航目标与当前位置基本重合，无需行驶')
            self._state = 'ARRIVED'
            self._pub_status.publish(String(data='ARRIVED'))
            return

        # 3) 初始化行驶状态
        self._pts = pts
        self._cum = [0.0]
        for i in range(1, len(pts)):
            self._cum.append(self._cum[-1] + dist2d(pts[i - 1], pts[i]))
        self._total_len = self._cum[-1]
        self._seg_idx = 0
        self._pid_e_prev = 0.0
        self._pid_ie = 0.0
        self._t_prev = None
        self._t_start = time.time()
        self._t_last_status = 0.0
        self._stats_max_e = 0.0
        self._stats_len = 0.0
        self._last_car = car
        self._state = 'DRIVING'
        self._publish_path()
        self._open_csv()

        speed = float(self.get_parameter('speed').value)
        self.get_logger().info(
            f'🚗 开始导航: 当前({car[0]:.2f},{car[1]:.2f}) → '
            f'终点({pts[-1][0]:.2f},{pts[-1][1]:.2f}) | '
            f'路径 {len(pts)} 点 / 总长 {self._total_len:.2f} m | 速度 {speed:.2f} m/s')

        # 已经在终点附近：直接完成
        if dist2d(car, pts[-1]) < float(self.get_parameter('goal_tolerance').value):
            self._arrive()

    # ----------------------------- 控制循环 -----------------------------
    def _control(self):
        if not self._have_odom:
            self.get_logger().info('⏳ 等待 /odom …（请确认仿真已启动: ~/start_car.sh）',
                                   throttle_duration_sec=5.0)
            self._publish(0.0, 0.0)
            return
        if self._state == 'DRIVING' and self._pts:
            self._follow()
        else:
            self._publish(0.0, 0.0)   # 非行驶状态持续发零速，防止溜车
            if self._pts and time.time() - self._t_path_pub > 3.0:
                self._publish_path()

    def _follow(self):
        g = self.get_parameter
        speed = float(g('speed').value)
        L_min = float(g('lookahead_min').value)
        L_max = float(g('lookahead_max').value)
        L_gain = float(g('lookahead_gain').value)
        kp = float(g('kp_lateral').value)
        ki = float(g('ki_lateral').value)
        kd = float(g('kd_lateral').value)
        corr_max = float(g('lateral_correction_max').value)
        max_omega = float(g('max_omega').value)
        tol = float(g('goal_tolerance').value)
        regulated = bool(g('regulated').value)

        x, y, yaw = self._car
        pts = self._pts

        # 1) 最近点投影（只能向前搜索，防止路径折返时回跳）
        d_now, seg_i, t_seg, projx, projy = self._nearest(x, y)

        # 2) 横向误差 e（车在路径左侧为正）+ 当前弧长
        p1, p2 = pts[seg_i], pts[seg_i + 1]
        seg_len = max(dist2d(p1, p2), 1e-6)
        tx, ty = (p2[0] - p1[0]) / seg_len, (p2[1] - p1[1]) / seg_len
        nx, ny = -ty, tx
        e = (x - projx) * nx + (y - projy) * ny
        self._stats_max_e = max(self._stats_max_e, abs(e))
        s_now = self._cum[seg_i] + t_seg * seg_len

        # 3) 前视点（Pure Pursuit 核心：取前方 L 处路径点为追踪目标）
        L = max(L_min, min(L_max, L_gain * speed))
        tgt = self._point_at_s(s_now + L)
        alpha = wrap_angle(math.atan2(tgt[1] - y, tgt[0] - x) - yaw)
        remain = dist2d((x, y), pts[-1])

        # 4) 目标在身后（|α| 很大）：先原地旋转对准路径方向，再正常循迹
        #    （否则 Pure Pursuit 在接近 180° 反向时会 sin(α)≈0 先直行画大弧）
        if abs(alpha) > float(g('spin_alpha_threshold').value):
            if self._scan_factor() <= 0.0:
                self._publish(0.0, 0.0)
                self.get_logger().warn('⚠️ 原地转向暂停：前方障碍物过近',
                                       throttle_duration_sec=1.0)
            else:
                self._publish(0.0, float(g('spin_rate').value)
                              * (1.0 if alpha > 0 else -1.0))
                self.get_logger().info('🔄 原地转向对准路径方向…',
                                       throttle_duration_sec=1.5)
            self._t_prev = time.time()
            self._pid_e_prev = e
            self._log_row(x, y, yaw, 0.0, 0.0, e, remain)
            return

        # 5) 曲率 → 角速度：ω = 2·v·sin(α) / L  （即 κ·v）
        omega_pp = 2.0 * speed * math.sin(alpha) / L

        # 6) PID 横向纠偏（叠加在纯追踪之上，加快回中线）
        now = time.time()
        dt = 0.05 if self._t_prev is None else max(0.01, min(0.5, now - self._t_prev))
        self._t_prev = now
        de = (e - self._pid_e_prev) / dt
        self._pid_ie = max(-0.5, min(0.5, self._pid_ie + e * dt))
        omega_corr = -(kp * e + ki * self._pid_ie + kd * de)
        omega_corr = max(-corr_max, min(corr_max, omega_corr))
        omega = omega_pp + omega_corr

        # 7) 速度整形
        v = speed
        if regulated:                       # 弯道自动减速（可选）
            v *= max(0.35, 1.0 - 0.9 * abs(alpha))
        if remain < 0.5:                    # 接近终点减速
            v *= max(0.25, remain / 0.5)

        # 8) 雷达安全保护
        guard = self._scan_factor()
        if guard <= 0.0:
            self._publish(0.0, 0.0)
            self.get_logger().warn('⚠️ 前方障碍物距离过近，停车保护中',
                                   throttle_duration_sec=1.0)
            self._log_row(x, y, yaw, 0.0, 0.0, e, remain)
            self._pid_e_prev = e
            return
        v *= guard

        # 9) 发布控制量
        omega = max(-max_omega, min(max_omega, omega))
        self._publish(v, omega)

        # 10) 记录
        self._stats_len += dist2d((x, y), self._last_car)
        self._last_car = (x, y)
        self._log_row(x, y, yaw, v, omega, e, remain)
        self._pid_e_prev = e

        # 11) 到达判定
        if remain < tol:
            self._arrive()
            return

        # 12) 状态提示（1Hz）
        if now - self._t_last_status > 1.0:
            self._t_last_status = now
            self.get_logger().info(
                f'[行驶] v={v:.2f} m/s | 横向误差={e:+.3f} m | 剩余={remain:.2f} m')
            self._pub_status.publish(String(
                data=f'DRIVING v={v:.2f} e={e:+.3f} remain={remain:.2f}'))

        # 13) 路径周期性重发（方便 RViz 中途打开也能看到）
        if now - self._t_path_pub > 3.0:
            self._publish_path()

    # ----------------------------- 工具函数 -----------------------------
    def _nearest(self, x, y):
        """在路径上找最近投影点。窗口向前搜索（防回退），都在窗口外则全局搜索。"""
        pts = self._pts
        nseg = len(pts) - 1
        lo = max(0, self._seg_idx - 1)
        hi = min(nseg - 1, self._seg_idx + 12)

        best = None
        for i in range(lo, hi + 1):
            p1, p2 = pts[i], pts[i + 1]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            l2 = dx * dx + dy * dy
            t = 0.0 if l2 < 1e-12 else ((x - p1[0]) * dx + (y - p1[1]) * dy) / l2
            t = max(0.0, min(1.0, t))
            px, py = p1[0] + dx * t, p1[1] + dy * t
            d = math.hypot(x - px, y - py)
            if best is None or d < best[0]:
                best = (d, i, t, px, py)
        if best[0] > 1.0:   # 窗口内都太远 → 全局重找（异常恢复）
            for i in range(nseg):
                p1, p2 = pts[i], pts[i + 1]
                dx, dy = p2[0] - p1[0], p2[1] - p1[1]
                l2 = dx * dx + dy * dy
                t = 0.0 if l2 < 1e-12 else ((x - p1[0]) * dx + (y - p1[1]) * dy) / l2
                t = max(0.0, min(1.0, t))
                px, py = p1[0] + dx * t, p1[1] + dy * t
                d = math.hypot(x - px, y - py)
                if d < best[0]:
                    best = (d, i, t, px, py)
        self._seg_idx = best[1]
        return best

    def _point_at_s(self, s):
        """按弧长 s 取路径上的点"""
        pts, cum = self._pts, self._cum
        if s <= 0.0:
            return pts[0]
        if s >= cum[-1]:
            return pts[-1]
        i = bisect.bisect_right(cum, s) - 1
        i = max(0, min(i, len(pts) - 2))
        seg_len = cum[i + 1] - cum[i]
        t = 0.0 if seg_len < 1e-9 else (s - cum[i]) / seg_len
        return (pts[i][0] + (pts[i + 1][0] - pts[i][0]) * t,
                pts[i][1] + (pts[i + 1][1] - pts[i][1]) * t)

    def _scan_factor(self):
        """雷达前向扇区检测：返回速度系数 (0=停车, 0.45=减速, 1=正常)"""
        if not bool(self.get_parameter('use_scan_guard').value):
            return 1.0
        if self._scan is None or time.time() - self._scan_t > 1.0:
            return 1.0
        slow = float(self.get_parameter('scan_slow_dist').value)
        stop = float(self.get_parameter('scan_stop_dist').value)
        half = math.radians(float(self.get_parameter('front_sector_deg').value))
        a0 = self._scan.angle_min
        da = self._scan.angle_increment
        factor = 1.0
        for k, r in enumerate(self._scan.ranges):
            ang = a0 + k * da
            if abs(wrap_angle(ang)) > half:
                continue
            if not math.isfinite(r) or r <= 0.0:
                continue
            if r < stop:
                return 0.0
            if r < slow:
                factor = min(factor, 0.45)
        return factor

    # ----------------------------- 发布 / 记录 -----------------------------
    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._pub_cmd.publish(msg)

    def _publish_path(self):
        if not self._pts:
            return
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for (px, py) in self._pts:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._pub_path.publish(msg)
        self._t_path_pub = time.time()

    def _open_csv(self):
        self._csv_f = None
        self._csv_w = None
        log_dir = str(self.get_parameter('csv_log_dir').value)
        if not log_dir:
            return
        try:
            log_dir = os.path.expanduser(log_dir)
            os.makedirs(log_dir, exist_ok=True)
            fn = os.path.join(log_dir, time.strftime('pp_run_%m%d_%H%M%S.csv'))
            self._csv_f = open(fn, 'w', newline='', encoding='utf-8')
            self._csv_w = csv.writer(self._csv_f)
            self._csv_w.writerow(['t', 'x', 'y', 'yaw_rad',
                                  'v_cmd', 'w_cmd', 'cross_err', 'remain'])
            # 同步记录本次规划路径（用于事后画图）
            with open(fn[:-4] + '.path', 'w', encoding='utf-8') as pf:
                for (px, py) in self._pts:
                    pf.write(f'{px:.4f} {py:.4f}\n')
            self.get_logger().info(f'运行记录: {fn}')
        except Exception as ex:
            self.get_logger().warn(f'无法写运行记录: {ex}')
            self._csv_f = None
            self._csv_w = None

    def _log_row(self, x, y, yaw, v, w, e, remain):
        if self._csv_w is None:
            return
        try:
            self._csv_w.writerow([f'{time.time() - self._t_start:.3f}',
                                  f'{x:.4f}', f'{y:.4f}', f'{yaw:.4f}',
                                  f'{v:.3f}', f'{w:.3f}',
                                  f'{e:+.4f}', f'{remain:.4f}'])
            self._csv_f.flush()
        except Exception:
            pass

    def _close_csv(self):
        if self._csv_f is not None:
            try:
                self._csv_f.close()
            except Exception:
                pass
        self._csv_f = None
        self._csv_w = None

    def _arrive(self):
        self._state = 'ARRIVED'
        self._publish(0.0, 0.0)
        x, y, _ = self._car
        dur = time.time() - self._t_start
        self.get_logger().info(
            f'✅ 到达终点 ({x:.2f}, {y:.2f})，停车\n'
            f'   用时 {dur:.1f} s | 行驶路程 {self._stats_len:.2f} m | '
            f'最大横向误差 {self._stats_max_e:.3f} m')
        self._pub_status.publish(String(data='ARRIVED'))
        self._publish_path()
        self._log_row(x, y, self._car[2], 0.0, 0.0, self._pid_e_prev, 0.0)
        self._close_csv()

    def stop_all(self):
        """退出前清理：停车 + 关文件"""
        for _ in range(5):
            self._publish(0.0, 0.0)
            time.sleep(0.02)
        self._close_csv()


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('收到 Ctrl+C，停车退出')
    finally:
        try:
            node.stop_all()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
