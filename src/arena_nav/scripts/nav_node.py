#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_nav::nav_node — 单机规划与轨迹跟踪（每机一个）。

输入: /drone_<id>/odom, /drone_<id>/planning/goal (PoseStamped)
输出: /drone_<id>/vel_cmd (Twist)
管线: goal → 2.5D A*（scene 占用 + inflation）→ 路径跟随 P 位置环 + 速度前馈
       → /vel_cmd → arena_world 动力学。
"""
import math

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

from zx2026_common import config as cfg
from zx2026_common.scene import Scene
from arena_nav.astar import AStar


class NavNode:
    def __init__(self):
        rospy.init_node("nav_node", anonymous=True)
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id

        rules = cfg.load("competition_rules.yaml")
        self.max_vel = float(rules["motion"].get("default_max_vel", 2.0))
        self.max_acc = float(rules["motion"].get("default_max_acc", 4.0))
        self.cruise_z = float(rules["heights"].get("cruise_z", 2.5))
        self.scene = Scene()
        settings = cfg.load("sim_settings.yaml")
        self.res = float(settings.get("nav", {}).get("resolution", 0.5))
        # 实际膨胀 = 机体半径 + 安全边距，确保路径不与树杆相切
        margin = float(settings.get("nav", {}).get("inflation", 0.4))
        self.inflation = self.scene.drone_radius + margin

        v = self.scene.venue
        self.astar = AStar(self.scene, resolution=self.res, inflation=self.inflation,
                           x_range=(-v["size"][0] / 2, v["size"][0] / 2),
                           y_range=(-v["size"][1] / 2, v["size"][1] / 2),
                           z_lo=0.5, z_hi=self.cruise_z + 1.0)

        self.odom = (0.0, 0.0, 1.0)
        self.path = []          # [(x,y,z),...]
        self.waypoint_idx = 0
        self.goal = None
        self.has_odom = False

        # 多机间距保持：订阅全部 odom，距离过近时叠加分离速度（防互撞）
        self.sep_radius = float(settings.get("nav", {}).get("separation_radius", 1.0))
        self.sep_gain = float(settings.get("nav", {}).get("separation_gain", 2.5))
        self.neighbors = {}

        self.pub_vel = rospy.Publisher(ns + "/vel_cmd", Twist, queue_size=10)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.Subscriber(ns + "/planning/goal", PoseStamped, self._on_goal)
        for j in range(self.scene.drone_count):
            if j == self.drone_id:
                continue
            rospy.Subscriber("/drone_%d/odom" % j, Odometry,
                             lambda m, j=j: self._on_neighbor(j, m))

        # 周期重规划（应对动态障碍/卡死）
        self.last_plan_t = -1.0
        rospy.loginfo("nav_node: drone %d max_vel=%.1f cruise_z=%.1f",
                      self.drone_id, self.max_vel, self.cruise_z)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom = (p.x, p.y, p.z)
        self.has_odom = True

    def _on_neighbor(self, j, msg):
        p = msg.pose.pose.position
        self.neighbors[j] = (p.x, p.y, p.z)

    def _on_goal(self, msg):
        p = msg.pose.position
        self.goal = (p.x, p.y, p.z)
        rospy.loginfo("nav_node: drone %d new goal (%.1f, %.1f, %.1f)",
                      self.drone_id, p.x, p.y, p.z)
        self._replan()

    def _replan(self):
        if self.goal is None:
            return
        path2 = self.astar.plan(self.odom[:2], self.goal[:2])
        # 航路点保持巡航高度；z 变化在 _tick 末段向 goal 收敛时处理
        self.path = [(x, y, self.cruise_z) for (x, y) in path2]
        self.waypoint_idx = 0
        self.last_plan_t = rospy.get_time()

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self):
        if not self.has_odom or self.goal is None:
            return
        # 定期重规划（每 2s）
        now = rospy.get_time()
        if now - self.last_plan_t > 2.0:
            self._replan()

        # 前进到当前航点
        pos = np.array(self.odom)
        cmd = np.zeros(3)
        if self.waypoint_idx < len(self.path):
            wp = np.array(self.path[self.waypoint_idx])
            err = wp - pos
            if math.hypot(err[0], err[1]) < 0.8:  # 水平距离判定
                self.waypoint_idx += 1
            if self.waypoint_idx < len(self.path):
                wp = np.array(self.path[self.waypoint_idx])
                err = wp - pos
            elif len(self.path) == 1:
                # 单点路径（如原地起飞，起终点同格）：向最终 goal 收敛（含 z 爬升）
                err = np.array(self.goal) - pos
            else:
                # 多航点路径已到末尾：向 goal 收敛（astar 已处理被占目标）
                err = np.array(self.goal) - pos

            # 位置环 P → 期望速度（限幅）
            vx = np.clip(err[0], -2.0, 2.0)
            vy = np.clip(err[1], -2.0, 2.0)
            vz = np.clip(err[2] * 1.5, -1.0, 1.0)
            cmd = np.array([vx, vy, vz])

            # 速度限幅
            vn = np.linalg.norm(cmd)
            if vn > self.max_vel:
                cmd = cmd * (self.max_vel / vn)

            # 到位判定
            if self.waypoint_idx >= len(self.path) and np.linalg.norm(err) < 0.4:
                cmd = np.zeros(3)
        else:
            # waypoint_idx 已越界（空路径兜底 / 单点路径 / 多航点已走完）
            if len(self.path) == 1:
                # 单点路径（如原地起飞，起终点同格）：向最终 goal 收敛（含 z 爬升）
                err = np.array(self.goal) - pos
            elif len(self.path) > 1:
                # 多航点路径已走完：向 goal 收敛
                err = np.array(self.goal) - pos
            else:
                # 空路径兜底：直接向 goal 收敛
                err = np.array(self.goal) - pos
            if np.linalg.norm(err) < 0.4:
                cmd = np.zeros(3)
            else:
                vx = np.clip(err[0], -2.0, 2.0)
                vy = np.clip(err[1], -2.0, 2.0)
                vz = np.clip(err[2] * 1.5, -1.0, 1.0)
                cmd = np.array([vx, vy, vz])
                vn = np.linalg.norm(cmd)
                if vn > self.max_vel:
                    cmd = cmd * (self.max_vel / vn)

        # 保持高度不低于地面 0.5m
        cmd[2] = max(cmd[2], (0.5 + self.scene.venue["ground_z"] - self.odom[2]) * 1.0)

        # 多机分离（软斥力 + 硬约束）：
        #   软斥力：邻居进入 sep_radius 内叠加斥力（3D，含高度差）。
        #   硬约束：邻居进入 hard_radius 内时剔除速度中朝向邻居的分量（速度障碍），
        #          并叠加随距离收敛的硬斥力，确保永不接近到 2*drone_radius(碰撞)。
        #   单纯软斥力在 d≈0.75 时仅 ~0.6，抵不过目标速度 2.0，穿越区/林缘狭缝
        #   仍会偶发接触->永久冻结；硬约束补上这一层。
        hard_r = 2.0 * self.scene.drone_radius + 1.1
        for j, nj in self.neighbors.items():
            dx = self.odom[0] - nj[0]
            dy = self.odom[1] - nj[1]
            dz = self.odom[2] - nj[2]
            d = math.sqrt(dx * dx + dy * dy + dz * dz)
            if d <= 0:
                continue
            if d < self.sep_radius:
                w = (self.sep_radius - d) / self.sep_radius
                cmd[0] += (dx / d) * self.sep_gain * w
                cmd[1] += (dy / d) * self.sep_gain * w
                cmd[2] += (dz / d) * self.sep_gain * w
            if d < hard_r:
                # 剔除朝向邻居的速度分量，越近叠加越大硬斥力
                ux, uy, uz = -dx / d, -dy / d, -dz / d   # 自己 -> 邻居 单位向量
                v_app = cmd[0] * ux + cmd[1] * uy + cmd[2] * uz
                if v_app > 0:
                    push = v_app + (hard_r - d) * 4.0
                    cmd[0] -= push * ux
                    cmd[1] -= push * uy
                    cmd[2] -= push * uz
        vn = np.linalg.norm(cmd)
        if vn > self.max_vel:
            cmd = cmd * (self.max_vel / vn)

        # 静态障碍避让（安全网，防止分离力把无人机侧推入树干）：
        #   A* 路径已远离树干，正常跟随不会触发；但多机硬分离是"障碍盲"的，可能
        #   把无人机以全速侧推入树 -> 永久冻结。此处补一层【动量感知】避让：
        #   用 odom 差分估计实际速度，沿速度方向前瞻"制动距离 v²/2a"，若该方向
        #   上将擦碰障碍则剔除朝向障碍的速度分量并叠加硬斥力（令其沿切向滑过）。
        #   前瞻沿速度方向：正常路径跟随时速度与树干相切，前瞻点距树干仍 ≥0.75m
        #   路径余量，不触发；仅当侧推令速度指向树干时才触发。低速时退化为 cmd
        #   短前瞻。fence/marker 同 A* 跳过（标识柱是投放目标，围栏在巡航高度外）。
        dr = self.scene.drone_radius
        now_v = rospy.get_time()
        if not hasattr(self, '_v_est'):
            self._v_est = np.zeros(3)
            self._prev_odom_v = np.array(self.odom)
            self._prev_odom_t = now_v
        else:
            dt_e = now_v - self._prev_odom_t
            if dt_e >= 0.04:
                self._v_est = 0.6 * self._v_est + 0.4 * \
                    (np.array(self.odom) - self._prev_odom_v) / max(dt_e, 1e-3)
                self._prev_odom_v = np.array(self.odom)
                self._prev_odom_t = now_v
        speed = float(np.linalg.norm(self._v_est))
        acc_cap = 6.0
        brake_dist = speed * speed / (2.0 * acc_cap)        # 制动距离
        look = brake_dist + dr + 0.15                        # 前瞻 = 制动距离 + 半径 + 余量
        if speed > 0.1:
            fx = self.odom[0] + (self._v_est[0] / speed) * look
            fy = self.odom[1] + (self._v_est[1] / speed) * look
            fz = self.odom[2] + (self._v_est[2] / speed) * look
        else:
            fx = self.odom[0] + cmd[0] * 0.3
            fy = self.odom[1] + cmd[1] * 0.3
            fz = self.odom[2] + cmd[2] * 0.3
        react_r = dr + 0.15          # 反应式兜底半径（< 0.75m 路径余量）
        pred_r = dr + 0.05           # 前瞻擦碰阈值
        px, py, pz = self.odom[0], self.odom[1], self.odom[2]
        for ob in self.scene.obstacles:
            if ob.kind in ("fence", "marker"):
                continue
            lo, hi = ob.lo, ob.hi
            if hi[2] < pz - dr or lo[2] > pz + dr:
                continue
            # 当前位置到障碍 AABB 最近点（用于投影方向 + 反应式触发）
            rx = min(max(px, lo[0]), hi[0])
            ry = min(max(py, lo[1]), hi[1])
            rz = min(max(pz, lo[2]), hi[2])
            ex, ey, ez = rx - px, ry - py, rz - pz
            ed = math.sqrt(ex * ex + ey * ey + ez * ez)
            # 前瞻点到障碍 AABB 最近点（用于预测触发）
            cx = min(max(fx, lo[0]), hi[0])
            cy = min(max(fy, lo[1]), hi[1])
            cz = min(max(fz, lo[2]), hi[2])
            ddx, ddy, ddz = cx - fx, cy - fy, cz - fz
            dd = math.sqrt(ddx * ddx + ddy * ddy + ddz * ddz)
            if ed >= react_r and dd >= pred_r:
                continue
            if ed <= 0:
                ex, ed = 1.0, 1e-3   # 已穿透 AABB：沿 +x 推出
            ux, uy, uz = ex / ed, ey / ed, ez / ed   # 自己 -> 障碍 单位向量
            v_app = cmd[0] * ux + cmd[1] * uy + cmd[2] * uz
            if v_app > 0:
                # 推力强度按更危险的（当前位置 ed / 前瞻点 dd）决定：
                # 当前近 -> 反应式硬斥力；前瞻近 -> 额外预测斥力抵消动量
                danger = min(ed, dd)
                push = v_app + max(0.0, react_r - danger) * 4.0 \
                       + max(0.0, pred_r - dd) * 6.0
                cmd[0] -= push * ux
                cmd[1] -= push * uy
                cmd[2] -= push * uz
        vn = np.linalg.norm(cmd)
        if vn > self.max_vel:
            cmd = cmd * (self.max_vel / vn)

        tw = Twist()
        tw.linear.x, tw.linear.y, tw.linear.z = cmd[0], cmd[1], cmd[2]
        self.pub_vel.publish(tw)

    # ---- 到达查询（供 mission 使用，通过公共话题/服务） ---------------------
    def goal_reached(self):
        if self.goal is None:
            return False
        return np.linalg.norm(np.array(self.goal) - np.array(self.odom)) < 0.4


if __name__ == "__main__":
    try:
        node = NavNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
