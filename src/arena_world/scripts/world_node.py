#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_world::world_node — 科目三 主仿真循环。

职责：
  * 解析 scene_topology/fleet/sim_settings，构建 Scene；
  * 在起降区 pads 生成 6 架 DroneSim（物理后端可切换）；
  * 订阅 /drone_<id>/vel_cmd，按 control_dt 步进全部动力学；
  * 每步碰撞检测（障碍 + 地面 + 六机互撞）；
  * 发布 /clock（sim time）、/drone_<id>/odom、
    /zx2026/scene（投放点 PoseArray）、/zx2026/markers（RViz）；
  * 服务 /zx2026/world/reset（重置到初始位姿）。
"""
import math
import random

import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (Twist, PoseArray, Pose, Point, Quaternion, Vector3,
                               Vector3Stamped, TransformStamped)
from tf.msg import tfMessage
from visualization_msgs.msg import Marker, MarkerArray

from zx2026_common import config as cfg
from zx2026_common import scene as sc
from zx2026_common import geometry as geo
from arena_world.dynamics import make_backend, DroneState
from arena_world.scene_markers import build_scene_messages


class DroneSim:
    def __init__(self, drone_id, pose, backend_name, settings):
        self.drone_id = drone_id
        self.backend = make_backend(backend_name, settings)
        self.backend.reset(pose)
        self.home = list(pose[:3])
        self.cmd_vel = np.zeros(3)
        self.cmd_yaw_rate = 0.0
        self.collided = False
        self._collide_logged = False
        # 碰撞恢复配置
        cc = settings.get("collision", {}).get("recovery", {})
        self._collision_enabled = bool(cc.get("enabled", True))
        self._bounce_dt = float(cc.get("bounce_dt", 1.0))
        self._cooldown_dt = float(cc.get("cooldown_dt", 2.0))
        self._bounce_vel = float(cc.get("bounce_vel", 2.0))
        self._max_collisions = int(cc.get("max_collisions", 3))
        self._collision_t = -1.0
        self._collision_count = 0
        self._bounce_dir = np.zeros(3)

    def set_vel_cmd(self, vx, vy, vz, yaw_rate):
        self.cmd_vel = np.array([vx, vy, vz])
        self.cmd_yaw_rate = yaw_rate

    def step(self, dt, wind=None):
        self.backend.set_vel_cmd(self.cmd_vel, self.cmd_yaw_rate)
        self.backend.step(dt, wind=wind)

    def state(self):
        return self.backend.get_state()


class WindModel:
    """风场模型：定常风 mean + 共享慢变阵风（零均值 OU）+ 每机独立湍流（零均值 OU）。

    OU 离散式：x += (mu-x)*dt/tau + sigma*sqrt(2*dt/tau)*N(0,1)，sigma 即稳态标准差。
    确定性：rng 由 run_seed 派生，每步抽取顺序固定（gx, gy, 各机 tx, ty）。
    仅水平分量（z 恒 0）：垂直由任务高度控制，不扰动。
    应用门控（world_node）：仅 P4_EXECUTE 飞行期（P3 起飞窗口不施风，见 _on_state 注释）；
    冷却冻结期跳过。
    """

    def __init__(self, params, seed, drone_count):
        mn = params.get("mean", [0.0, 0.0])
        self.mean = [float(mn[0]), float(mn[1])]
        self.gust_std = float(params.get("gust_std", 0.4))
        self.gust_tau = float(params.get("gust_timescale_s", 4.0))
        self.turb_std = float(params.get("turb_std", 0.25))
        self.turb_tau = float(params.get("turb_timescale_s", 1.0))
        self.rng = random.Random(seed + 0xA11CE)
        self.gx = 0.0
        self.gy = 0.0
        self.tx = [0.0] * drone_count
        self.ty = [0.0] * drone_count

    def _ou(self, x, mu, std, tau, dt):
        return (x + (mu - x) * dt / tau
                + std * math.sqrt(2.0 * dt / tau) * self.rng.gauss(0.0, 1.0))

    def step(self, dt):
        self.gx = self._ou(self.gx, 0.0, self.gust_std, self.gust_tau, dt)
        self.gy = self._ou(self.gy, 0.0, self.gust_std, self.gust_tau, dt)
        for i in range(len(self.tx)):
            self.tx[i] = self._ou(self.tx[i], 0.0, self.turb_std, self.turb_tau, dt)
            self.ty[i] = self._ou(self.ty[i], 0.0, self.turb_std, self.turb_tau, dt)

    def wind_for(self, i):
        return np.array([self.mean[0] + self.gx + self.tx[i],
                         self.mean[1] + self.gy + self.ty[i],
                         0.0])

    def reset(self):
        self.gx = 0.0
        self.gy = 0.0
        self.tx = [0.0] * len(self.tx)
        self.ty = [0.0] * len(self.ty)


class WorldNode:
    def __init__(self):
        rospy.init_node("world_node", anonymous=False)

        self.sim_settings = cfg.load("sim_settings.yaml")
        self.scene = sc.Scene()

        self.use_sim_time = bool(self.sim_settings.get("use_sim_time", True))
        self.world_dt = float(self.sim_settings.get("world_dt", 0.01))
        self.control_dt = float(self.sim_settings.get("control_dt", 0.05))
        self.publish_dt = float(self.sim_settings.get("publish_dt", 0.05))
        self.backend_name = self.sim_settings.get("backend", "cascade_pid")

        # 风场模型（enabled:false → None → 所有路径 wind=None，零侵入）
        wd = self.sim_settings.get("wind", {})
        self.wind_model = None
        if bool(wd.get("enabled", False)):
            self.wind_model = WindModel(wd, int(self.sim_settings.get("run_seed", 42)),
                                        self.scene.drone_count)
        self.mission_active = False
        self._last_state = None
        rospy.Subscriber("/zx2026/state", String, self._on_state)

        # 起降区 pads → 6 机初始位姿
        pads = self.scene.get_pads()
        self.drones = {}
        for i, (px, py) in enumerate(pads):
            if i >= self.scene.drone_count:
                break
            dz = float(self.scene.venue["ground_z"]) + 1.0
            pose = (px, py, dz, 0.0)
            self.drones[i] = DroneSim(i, pose, self.backend_name, self.sim_settings)

        # ---- 话题 ----
        self.pub_clock = rospy.Publisher("/clock", Clock, queue_size=1, latch=True)
        self.pub_scene = rospy.Publisher("/zx2026/scene", PoseArray, queue_size=1, latch=True)
        self.pub_markers = rospy.Publisher("/zx2026/markers", MarkerArray, queue_size=1, latch=True)
        self.pub_odom = {}
        self.pub_collision = {}
        self.sub_vel = {}
        self.landing_armed = {i: False for i in self.drones}
        _rc = self.sim_settings.get("collision", {}).get("recovery", {})
        # 恢复期指令隔离（lock_cmd）：弹开/冷却/冻结指令独占 cmd_vel。
        # 执行器 20Hz 指令流会在 world 步进间隙覆盖恢复指令——架空永久
        # 冻结后 hover 回拉持续下压 + collided 期间免碰撞检测 → 落地机
        # 无限下沉（run4/5 法证：d1 沉至 z=-3.56、d0 -0.70 误判退赛）
        self._recovery_lock = bool(_rc.get("lock_cmd", True))
        # 停机豁免（park_ground_skip）：armed 过（计划着地序列）即永久
        # 跳过地面子句——touchdown 解除 armed 时 z≈0.27 仍在空中，触地
        # 子句会把着地机判成碰撞弹跳（run4 五机簇2 法证）。退赛监视
        # （rule_monitor）不受此豁免影响，仍按 z/pad/armed 独立判定。
        self._park_skip = bool(_rc.get("park_ground_skip", True))
        # 低空弹开钳位（low_bounce_up）：z<1.0 时弹开方向垂直分量一律向上——
        # 爬升中被撞（rev 含向下分量）会把机压进地面（run6 法证二次触地
        # z=0.01 级联）；互撞弹开本就水平，不受影响
        self._low_bounce_up = bool(_rc.get("low_bounce_up", True))
        self._ever_armed = {i: False for i in self.drones}
        for i, d in self.drones.items():
            ns = "/drone_%d" % i
            self.pub_odom[i] = rospy.Publisher(ns + "/odom", Odometry, queue_size=10)
            self.pub_collision[i] = rospy.Publisher(ns + "/collision", Bool, queue_size=1, latch=True)
            self.sub_vel[i] = rospy.Subscriber(
                ns + "/vel_cmd", Twist, lambda msg, i=i: self._on_vel_cmd(i, msg))
            # 真落地门控：armed=True 时跳过地面子句（最后下降段才有生存期，
            # touchdown 后立即复位——漏关放过不了树/围栏/机间碰撞）
            rospy.Subscriber(ns + "/mission/landing_armed", Bool,
                             lambda msg, i=i: self._on_landing_armed(i, msg))
        # TF：world → drone_<i>，供 RViz 显示无人机位姿/轨迹
        self.tf_pub = rospy.Publisher("/tf", tfMessage, queue_size=10)
        # 风观测（仅启用时发布，供 flock_obs 采样阵风幅值）
        self.pub_wind = None
        if self.wind_model is not None:
            self.pub_wind = rospy.Publisher("/zx2026/wind", Vector3Stamped, queue_size=10)

        # ---- 服务 ----
        self.srv_reset = rospy.Service("/zx2026/world/reset", Empty, self._on_reset)

        # 模拟时钟
        self.sim_t = 0.0
        self._acc_control = 0.0
        self._acc_pub = 0.0

        rospy.loginfo("world_node: %d drones, backend=%s, dt=%s",
                      len(self.drones), self.backend_name, self.world_dt)
        self._publish_scene_static()

    # ---------------------------------------------------------------- callbacks
    def _on_state(self, msg):
        st = msg.data
        if st != self._last_state:
            self._last_state = st
            if st == "P3_TAKEOFF" and self.wind_model is not None:
                # 任务开始：阵风从 0 起，保证确定性可复现
                self.wind_model.reset()
        # 风只在 P4_EXECUTE（穿越/投放/返航，含返航）生效；P1/P2 停靠、P5 后不施风。
        # P3 起飞窗口不施风：pads 仅 1.5m 间隔 + 错峰起飞是全系统最脆弱的碰撞窗口，
        # 实测风漂移（悬停 0.455·w）会在此引发碰撞级联拖垮任务（R1 验证结论）。
        # 起飞是贴地受控阶段，风主要影响 P4 开放空域（A* 偏差/投放精度/群集对齐）。
        self.mission_active = st == "P4_EXECUTE"

    def _on_vel_cmd(self, i, msg):
        d = self.drones[i]
        if self._recovery_lock and d.collided and d._collision_enabled:
            # 碰撞恢复期丢弃执行器指令：恢复序列（弹开/冷却/冻结）必须
            # 完整执行，否则被 20Hz 指令流逐拍覆盖架空（见 __init__ 注释）
            return
        d.set_vel_cmd(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

    def _on_landing_armed(self, i, msg):
        prev = self.landing_armed.get(i, False)
        self.landing_armed[i] = bool(msg.data)
        if msg.data:
            self._ever_armed[i] = True  # 停机豁免锁存（_park_skip 用）
        if prev and not msg.data:
            rospy.loginfo("drone %d landing disarmed", i)
        elif msg.data and not prev:
            rospy.loginfo("drone %d landing armed", i)

    def _on_reset(self, req):
        for i, d in self.drones.items():
            d.backend.reset((d.home[0], d.home[1], d.home[2], 0.0))
            self.landing_armed[i] = False
            self._ever_armed[i] = False
            d.collided = False
            d._collide_logged = False
            d.cmd_vel = np.zeros(3)
            d._collision_t = -1.0
            d._collision_count = 0
            d._bounce_dir = np.zeros(3)
            self.pub_collision[i].publish(Bool(data=False))
        if self.wind_model is not None:
            self.wind_model.reset()
        self.sim_t = 0.0
        return EmptyResponse()

    # ---------------------------------------------------------------- main loop
    def run(self):
        rate = rospy.Rate(1.0 / self.world_dt)
        while not rospy.is_shutdown():
            self.step_once()
            rate.sleep()

    def step_once(self):
        dt = self.world_dt
        self.sim_t += dt
        if self.wind_model is not None:
            self.wind_model.step(dt)

        # 碰撞检测（对上一状态）：障碍 + 地面（landing_armed 时机豁免地面子句）
        for i, d in self.drones.items():
            if d.collided:
                continue
            p = d.state().pos_tuple()
            skip_g = self.landing_armed.get(i, False)
            if self._park_skip and self._ever_armed.get(i, False):
                skip_g = True  # 停机豁免：计划着地机贴地不判碰撞
            if self.scene.collides(p, skip_ground=skip_g):
                d.collided = True
                self.pub_collision[i].publish(Bool(data=True))
                if not d._collide_logged:
                    d._collide_logged = True
                    rospy.logwarn("drone %d COLLIDED obstacle at (%.2f,%.2f,%.2f)",
                                  i, p[0], p[1], p[2])
                # 碰撞恢复：记录时间、计算弹开方向
                if d._collision_t < 0:
                    d._collision_t = self.sim_t
                    d._collision_count += 1
                    speed = float(np.linalg.norm(d.cmd_vel))
                    if speed > 0.01:
                        rev = (-d.cmd_vel[0], -d.cmd_vel[1], -d.cmd_vel[2])
                        d._bounce_dir = np.array(geo.normalize(rev)) * d._bounce_vel
                    else:
                        d._bounce_dir = np.array([0.0, 0.0, d._bounce_vel])
                    if self._low_bounce_up and p[2] < 1.0 and d._bounce_dir[2] < 0.0:
                        # 低空弹开钳位：见 __init__ 注释
                        d._bounce_dir[2] = -d._bounce_dir[2]

        # 六机互撞
        states = {i: d.state() for i, d in self.drones.items()}
        for i, si in states.items():
            if self.drones[i].collided:
                continue
            for j, sj in states.items():
                if i >= j:
                    continue
                if geo.norm2((si.pos[0] - sj.pos[0], si.pos[1] - sj.pos[1],
                              si.pos[2] - sj.pos[2])) <= (2 * self.scene.drone_radius) ** 2:
                    self.drones[i].collided = True
                    self.drones[j].collided = True
                    self.pub_collision[i].publish(Bool(data=True))
                    self.pub_collision[j].publish(Bool(data=True))
                    for k in (i, j):
                        if not self.drones[k]._collide_logged:
                            self.drones[k]._collide_logged = True
                            q = self.drones[k].state().pos_tuple()
                            rospy.logwarn("drone %d COLLIDED inter-drone with %d at (%.2f,%.2f,%.2f)",
                                          k, (j if k == i else i), q[0], q[1], q[2])
                        # 碰撞恢复：首次碰撞记录弹开方向（远离对方）
                        dk = self.drones[k]
                        if dk._collision_t < 0:
                            other = i if k == j else j
                            away = (q[0] - states[other].pos[0],
                                    q[1] - states[other].pos[1], 0.0)
                            dk._collision_t = self.sim_t
                            dk._collision_count += 1
                            dk._bounce_dir = np.array(geo.normalize(away)) * dk._bounce_vel

        # 步进（碰撞恢复：弹开 → 冷却 → 恢复，替代永久冻结）
        for d in self.drones.values():
            if d.collided:
                if not d._collision_enabled:
                    d.cmd_vel = np.zeros(3)  # 原行为：永久冻结
                else:
                    elapsed = self.sim_t - d._collision_t
                    if d._collision_count > d._max_collisions:
                        d.cmd_vel = np.zeros(3)  # 超限永久冻结
                    elif elapsed < d._bounce_dt:
                        d.cmd_vel = d._bounce_dir  # 弹开
                    elif elapsed < d._bounce_dt + d._cooldown_dt:
                        d.cmd_vel = np.zeros(3)  # 冷却冻结
                    else:
                        # 恢复：清除碰撞状态，让 nav 重新接管
                        d.collided = False
                        d._collide_logged = False
                        d._collision_t = -1.0
                        self.pub_collision[d.drone_id].publish(Bool(data=False))
                        rospy.loginfo("drone %d recovered from collision (count=%d)",
                                      d.drone_id, d._collision_count)
            wind_vec = None
            if self.wind_model is not None and self.mission_active:
                # 冷却冻结期跳过风：避免碰撞点漂移导致碰撞计数升级为永久冻结
                in_cooldown = (d.collided and d._collision_enabled
                               and d._bounce_dt <= self.sim_t - d._collision_t
                               < d._bounce_dt + d._cooldown_dt)
                if not in_cooldown:
                    wind_vec = self.wind_model.wind_for(d.drone_id)
            d.step(dt, wind=wind_vec)

        # 时钟发布（每 control tick 同步一次 /clock）
        self._acc_pub += dt
        if self._acc_pub >= self.publish_dt:
            self._acc_pub = 0.0
            self._publish_clock_and_odom()

    # ---------------------------------------------------------------- publishing
    def _publish_clock_and_odom(self):
        if self.use_sim_time:
            c = Clock()
            c.clock.secs = int(self.sim_t)
            c.clock.nsecs = int((self.sim_t - int(self.sim_t)) * 1e9)
            self.pub_clock.publish(c)
        # 风观测：共享分量（定常风 + 阵风，不含每机湍流），供 flock_obs 采样 +
        # nav 风前馈。仅 mission_active（P4 飞行期）发真实值，其余发 0——OU 在非
        # 飞行期仍演化，若发布非零而无人机未受力，nav 前馈会朝"幻影风"漂。
        if self.pub_wind is not None:
            w = Vector3Stamped()
            w.header.stamp = rospy.Time.now()
            w.header.frame_id = "world"
            if self.mission_active:
                w.vector.x = self.wind_model.mean[0] + self.wind_model.gx
                w.vector.y = self.wind_model.mean[1] + self.wind_model.gy
            self.pub_wind.publish(w)
        for i, d in self.drones.items():
            od = Odometry()
            od.header.stamp = rospy.Time.now()
            od.header.frame_id = "world"
            od.child_frame_id = "drone_%d" % i
            s = d.state()
            od.pose.pose.position = Point(s.pos[0], s.pos[1], s.pos[2])
            od.pose.pose.orientation = quat_from_yaw(s.yaw)
            od.twist.twist.linear.x = s.vel[0]
            od.twist.twist.linear.y = s.vel[1]
            od.twist.twist.linear.z = s.vel[2]
            self.pub_odom[i].publish(od)
            # 世界系→机体 TF（位姿 + yaw）
            tr = TransformStamped()
            tr.header.stamp = od.header.stamp
            tr.header.frame_id = "world"
            tr.child_frame_id = "drone_%d" % i
            tr.transform.translation = Vector3(s.pos[0], s.pos[1], s.pos[2])
            tr.transform.rotation = quat_from_yaw(s.yaw)
            self.tf_pub.publish(tfMessage(transforms=[tr]))

    def _publish_scene_static(self):
        pa, ma = build_scene_messages(self.scene)
        pa.header.stamp = rospy.Time.now()
        self.pub_scene.publish(pa)
        self.pub_markers.publish(ma)
        rospy.loginfo("world_node: scene published (%d drop points, %d markers)",
                      len(self.scene.drop_points), len(ma.markers))


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


if __name__ == "__main__":
    try:
        node = WorldNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
