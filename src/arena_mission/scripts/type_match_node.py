#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_mission::type_match_node — 全局匹配确认 + 悬停安全策略（全局 1 份）。

科目三两模式（competition_rules.color_id.enabled）：
- true（闭环）: expected=本机箱色 u8（box_color，空则回退 color_map[payload]），
  检测源=/drone_<i>/detected/color；期望位置=执行器 RECON 锁定的平台
  （/drone_<i>/mission/selected_dp 事件）。MATCH 语义="在锁定平台正上方
  复核 平台色==箱色" 的机上二次确认。
- false（fallback）: expected=载荷类型 u8，检测源=/drone_<i>/detected/type；
  期望位置=任务参考投放点（旧行为）。
安全策略（旧平台思路，重写）：P1 置信度门槛、P2 连续帧、P3 悬停超时、
  P4 未知忽略、P5 位置门限（期望位置超 drop_arrival_tol 的帧不计不清零——
  修复历史死代码：position_gate 参数此前读了未用）。
"""
import rospy
from std_msgs.msg import String, Bool, UInt8, Float32, Int32
from nav_msgs.msg import Odometry

from zx2026_common import config as cfg
from zx2026_common import geometry as geo
from zx2026_common.scene import Scene
from zx2026_common.msg import Mission


class PerDroneMatcher:
    def __init__(self, drone_id, params, gate_tol):
        self.drone_id = drone_id
        self.min_conf = float(params.get("min_confidence", 0.8))
        self.n_frames = int(params.get("consecutive_frames", 3))
        self.timeout = float(params.get("hover_timeout_s", 12.0))
        self.position_gate = bool(params.get("position_gate", True))
        self.gate_tol = float(gate_tol)
        self.expected = None       # u8（闭环=箱色 / fallback=载荷类型）
        self.drop_xyz = None       # 位置门限目标（闭环=锁定平台 / fallback=参考平台）
        self.odom = (0.0, 0.0, 1.0)
        self.state = "IDLE"        # IDLE / HOVER
        self.consecutive = 0
        self.hover_start = None
        self.matched = False

    def on_mission(self, expected_u8, drop_xyz):
        self.expected = expected_u8
        self.drop_xyz = drop_xyz
        self.reset()

    def on_selected(self, drop_xyz):
        """闭环 RECON 锁平台事件：位置门限目标切换到锁定平台。"""
        self.drop_xyz = drop_xyz

    def on_at_drop(self, active):
        if active and self.state == "IDLE":
            self.state = "HOVER"
            self.consecutive = 0
            self.hover_start = rospy.get_time()
        elif not active:
            self.reset()

    def on_detection(self, det_u8, conf):
        if self.state != "HOVER" or self.matched:
            return None
        if self.expected is None:
            return None
        # P5 位置门限（真正生效）：不在期望平台 tol 内的帧不计不清零
        if self.position_gate and self.drop_xyz is not None:
            if geo.dist_xy(self.odom, self.drop_xyz) > self.gate_tol:
                return None
        if det_u8 == self.expected and conf >= self.min_conf:
            self.consecutive += 1
            if self.consecutive >= self.n_frames:
                self.matched = True
                self.state = "IDLE"
                return "MATCH"
        else:
            self.consecutive = 0
        return None

    def on_tick(self, now):
        if self.state == "HOVER" and not self.matched:
            if self.hover_start and (now - self.hover_start) > self.timeout:
                self.reset()
                return "TIMEOUT"
        return None

    def reset(self):
        self.state = "IDLE"
        self.consecutive = 0
        self.hover_start = None


class TypeMatchNode:
    def __init__(self):
        rospy.init_node("type_match_node", anonymous=True)
        rules = cfg.load("competition_rules.yaml")
        params = rules.get("type_match", {})
        gate_tol = float(rules.get("heights", {}).get("drop_arrival_tol", 0.8))
        self._cid_enabled = bool(rules.get("color_id", {}).get("enabled", False))
        self.scene = Scene()
        self.n_drones = self.scene.drone_count

        self.matchers = {i: PerDroneMatcher(i, params, gate_tol)
                         for i in range(self.n_drones)}
        self.pub = {i: rospy.Publisher("/drone_%d/match/result" % i, String, queue_size=1)
                    for i in range(self.n_drones)}

        self.det_conf = {}
        for i in range(self.n_drones):
            ns = "/drone_%d" % i
            rospy.Subscriber(ns + "/detected/type", UInt8,
                             lambda m, i=i: self._on_det_type(i, m))
            rospy.Subscriber(ns + "/detected/color", UInt8,
                             lambda m, i=i: self._on_det_color(i, m))
            rospy.Subscriber(ns + "/detected/confidence", Float32,
                             lambda m, i=i: self._set_conf(i, m.data))
            rospy.Subscriber(ns + "/mission/at_drop", Bool,
                             lambda m, i=i: self._on_at_drop(i, m))
            rospy.Subscriber(ns + "/mission/selected_dp", Int32,
                             lambda m, i=i: self._on_selected(i, m))
            rospy.Subscriber(ns + "/odom", Odometry,
                             lambda m, i=i: self._on_odom(i, m))
            rospy.Subscriber("/zx2026/mission/%d" % i, Mission,
                             lambda m, i=i: self._on_mission(i, m))

        self._pub_result(0, "NONE")
        rospy.loginfo("type_match_node: %d drones (mode=%s)",
                      self.n_drones,
                      "closed-loop/color" if self._cid_enabled else "fallback/type")

    def _on_mission(self, i, msg):
        if self._cid_enabled:
            box = msg.box_color or self.scene.color_for_type(msg.payload_type)
            expected = cfg.color_to_uint8(box)
        else:
            expected = cfg.type_to_uint8(msg.payload_type)
        drop_xyz = None
        for dp in self.scene.drop_points:
            if dp.id == msg.drop_point_id:
                drop_xyz = (dp.xyz[0], dp.xyz[1], dp.xyz[2])
        self.matchers[i].on_mission(expected, drop_xyz)

    def _on_selected(self, i, msg):
        dp = self.scene.drop_point(msg.data)
        if dp is not None:
            self.matchers[i].on_selected((dp.xyz[0], dp.xyz[1], dp.xyz[2]))

    def _on_odom(self, i, msg):
        p = msg.pose.pose.position
        self.matchers[i].odom = (p.x, p.y, p.z)

    def _on_at_drop(self, i, msg):
        self.matchers[i].on_at_drop(msg.data)

    def _on_det_type(self, i, msg):
        if self._cid_enabled:
            return
        self._on_det(i, msg.data)

    def _on_det_color(self, i, msg):
        if not self._cid_enabled:
            return
        self._on_det(i, msg.data)

    def _on_det(self, i, det_u8):
        conf = self.det_conf.get(i, 0.0)
        res = self.matchers[i].on_detection(det_u8, conf)
        if res == "MATCH":
            self._pub_result(i, "MATCH")
            rospy.loginfo("drone %d MATCH confirmed, releasing", i)

    def _set_conf(self, i, conf):
        self.det_conf[i] = conf

    def _pub_result(self, i, val):
        self.pub[i].publish(String(data=val))

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            now = rospy.get_time()
            for i, m in self.matchers.items():
                res = m.on_tick(now)
                if res == "TIMEOUT":
                    self._pub_result(i, "TIMEOUT")
                    rospy.logwarn("drone %d HOVER_TIMEOUT", i)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = TypeMatchNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
