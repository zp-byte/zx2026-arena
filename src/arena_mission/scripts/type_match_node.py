#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_mission::type_match_node — 全局类型匹配 + 悬停安全策略（全局 1 份）。

订阅 6 机检测结果 + at_drop 信号，每机独立状态机：
  挂起计数 → 连续 N 帧命中预期类型（置信度≥门槛）→ 下发 MATCH
  悬停超时未匹配 → 下发 TIMEOUT（执行器重试/失败）
安全策略（旧平台思路，重写）：P1 置信度门槛、P2 连续帧、P3 悬停超时、
  P4 未知 tag 忽略、P5 位置门限（由执行器到达 identify 点保证）。
"""
import rospy
from std_msgs.msg import String, Bool, UInt8, Float32, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from zx2026_common import config as cfg
from zx2026_common.scene import Scene
from zx2026_common.msg import Mission


class PerDroneMatcher:
    def __init__(self, drone_id, params):
        self.drone_id = drone_id
        self.min_conf = float(params.get("min_confidence", 0.8))
        self.n_frames = int(params.get("consecutive_frames", 3))
        self.timeout = float(params.get("hover_timeout_s", 12.0))
        self.position_gate = bool(params.get("position_gate", True))
        self.expected = None       # TYPE_A..
        self.drop_xyz = None
        self.state = "IDLE"        # IDLE / HOVER
        self.consecutive = 0
        self.hover_start = None
        self.matched = False

    def on_mission(self, msg, drop_xyz):
        self.expected = msg.payload_type
        self.drop_xyz = drop_xyz
        self.reset()

    def on_at_drop(self, active):
        if active and self.state == "IDLE":
            self.state = "HOVER"
            self.consecutive = 0
            self.hover_start = rospy.get_time()
        elif not active:
            self.reset()

    def on_detection(self, det_type, conf):
        if self.state != "HOVER" or self.matched:
            return None
        if self.expected is None:
            return None
        if det_type == cfg.type_to_uint8(self.expected) and conf >= self.min_conf:
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
        self.scene = Scene()
        self.n_drones = self.scene.drone_count

        self.matchers = {i: PerDroneMatcher(i, params) for i in range(self.n_drones)}
        self.pub = {i: rospy.Publisher("/drone_%d/match/result" % i, String, queue_size=1)
                    for i in range(self.n_drones)}

        self.det_type = {}
        self.det_conf = {}
        for i in range(self.n_drones):
            ns = "/drone_%d" % i
            rospy.Subscriber(ns + "/detected/type", UInt8,
                             lambda m, i=i: self._on_det(i, m))
            rospy.Subscriber(ns + "/detected/confidence", Float32,
                             lambda m, i=i: self._set_conf(i, m.data))
            rospy.Subscriber(ns + "/mission/at_drop", Bool,
                             lambda m, i=i: self._on_at_drop(i, m))
            rospy.Subscriber("/zx2026/mission/%d" % i, Mission,
                             lambda m, i=i: self._on_mission(i, m))

        self._pub_result(0, "NONE")
        rospy.loginfo("type_match_node: %d drones", self.n_drones)

    def _on_mission(self, i, msg):
        drop_xyz = None
        for dp in self.scene.drop_points:
            if dp.id == msg.drop_point_id:
                drop_xyz = (dp.xyz[0], dp.xyz[1], dp.xyz[2])
        self.matchers[i].on_mission(msg, drop_xyz)

    def _on_at_drop(self, i, msg):
        self.matchers[i].on_at_drop(msg.data)

    def _on_det(self, i, msg):
        conf = self.det_conf.get(i, 0.0)
        self.det_type[i] = msg.data
        res = self.matchers[i].on_detection(msg.data, conf)
        if res == "MATCH":
            self._pub_result(i, "MATCH")
            rospy.loginfo("drone %d type MATCH confirmed, releasing", i)

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
