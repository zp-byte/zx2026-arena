#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_mission::mission_executor_node — 单机任务执行（每机一个）。

输入: /drone_<id>/odom, /zx2026/mission/<drone_id> (Mission),
      /zx2026/state (String), /drone_<id>/match/result (String),
      /drone_<id>/payload/done (Bool)
输出: /drone_<id>/planning/goal (PoseStamped),
      /drone_<id>/mission/phase (String),
      /drone_<id>/mission/at_drop (Bool),
      /drone_<id>/payload/command (UInt8),
      /drone_<id>/mission/crossed_zone (Bool)
状态机: IDLE → TAKEOFF → TAKEOFF_DONE → EXECUTE_PENDING → EXECUTE → AT_DROP → DROP → RETURN → DONE/FAILED
"""
import math

import rospy
import numpy as np
from std_msgs.msg import String, Bool, UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from zx2026_common import config as cfg
from zx2026_common import scene as sc
from zx2026_common.scene import Scene
from zx2026_common.msg import Mission


class MissionExecutor:
    def __init__(self):
        rospy.init_node("mission_executor_node", anonymous=True)
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id

        self.scene = Scene()
        # 非凸-α 编队协调层：本机相对共享航点的偏移（None = 未开启）
        self.formation_offset = self._compute_formation_offset()
        rules = cfg.load("competition_rules.yaml")
        h = rules["heights"]
        self.takeoff_hover_z = float(h.get("takeoff_hover_z", 1.5))
        self.cruise_z = float(h.get("cruise_z", 2.5))
        self.identify_z = float(h.get("identify_z", 1.2))
        self.arrival_tol = float(h.get("area_arrival_tol", 2.0))
        self.drop_tol = float(h.get("drop_arrival_tol", 0.8))
        self.max_vel = float(rules["motion"].get("default_max_vel", 2.0))
        # 多机错峰起飞：drone i 延迟 i*stagger 秒，避免六机同挤入口
        self.stagger = float(rules.get("takeoff_stagger_s", 3.0))
        self.takeoff_at = None
        # P4 进入林区同样错峰：P4 触发后 drone i 再延迟 i*entry_stagger 秒出发，
        # 防止六机同时涌入穿越区入口（入口窄、A* 路径在此汇聚导致互撞）
        self.entry_stagger = float(rules.get("entry_stagger_s", 2.5))
        self.enter_at = None
        # 返航同样错峰：投放完成后 drone i 延迟 i*return_stagger 秒再返航，
        # 防止六机同时涌入返航走廊（y≈6~7 林缘狭缝）互撞被永久冻结
        self.return_stagger = float(rules.get("return_stagger_s", 3.0))
        self.return_at = None

        self.state = "IDLE"
        self.mission = None
        self.goal = None
        self.odom = (0.0, 0.0, 1.0)
        self.pad = (0.0, 0.0)
        self.drop = (0.0, 0.0)
        self.retry = 0
        self.max_retry = 2

        # 穿越区强制通过
        self.crossed_zone = False

        self.pub_goal = rospy.Publisher(ns + "/planning/goal", PoseStamped, queue_size=10)
        self.pub_phase = rospy.Publisher(ns + "/mission/phase", String, queue_size=1, latch=True)
        self.pub_at_drop = rospy.Publisher(ns + "/mission/at_drop", Bool, queue_size=1, latch=True)
        self.pub_payload_cmd = rospy.Publisher(ns + "/payload/command", UInt8, queue_size=10)
        self.pub_crossed = rospy.Publisher(ns + "/mission/crossed_zone", Bool, queue_size=1, latch=True)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.Subscriber("/zx2026/mission/%d" % self.drone_id, Mission, self._on_mission)
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.Subscriber(ns + "/match/result", String, self._on_match)
        rospy.Subscriber(ns + "/payload/done", Bool, self._on_payload_done)

        self._publish_phase("IDLE")
        self.pub_crossed.publish(Bool(data=False))
        rospy.loginfo("mission_executor_node: drone %d", self.drone_id)

    # ---------------------------------------------------------------- callbacks
    def _on_odom(self, msg):
        self.odom = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     msg.pose.pose.position.z)

    def _on_mission(self, msg):
        if self.state != "IDLE":
            return
        self.mission = msg
        pads = self.scene.get_pads()
        if self.drone_id < len(pads):
            self.pad = pads[self.drone_id]
        else:
            self.pad = (0.0, 0.0)
        for dp in self.scene.drop_points:
            if dp.id == msg.drop_point_id:
                self.drop = (dp.xyz[0], dp.xyz[1])
        self.crossed_zone = False
        self.pub_crossed.publish(Bool(data=False))
        rospy.loginfo("drone %d got mission: payload=%s drop_point=%d",
                      self.drone_id, msg.payload_type, msg.drop_point_id)

    def _on_state(self, msg):
        st = msg.data
        if st == "P3_TAKEOFF" and self.state == "IDLE":
            self.takeoff_at = rospy.get_time() + self.drone_id * self.stagger
            self.state = "TAKEOFF_PENDING"
            rospy.loginfo("drone %d takeoff scheduled in %.1fs", self.drone_id,
                          self.drone_id * self.stagger)
        elif st == "P4_EXECUTE" and self.state == "TAKEOFF_DONE":
            self.enter_at = rospy.get_time() + self.drone_id * self.entry_stagger
            self.state = "EXECUTE_PENDING"
            self._publish_phase("EXECUTE_PENDING")
            rospy.loginfo("drone %d execute entry scheduled in %.1fs",
                          self.drone_id, self.drone_id * self.entry_stagger)

    def _on_match(self, msg):
        if self.state == "AT_DROP" and msg.data == "MATCH":
            self.state = "DROP"
            self._publish_phase("DROP")
            self.pub_payload_cmd.publish(UInt8(data=cfg.type_to_uint8(self.mission.payload_type)))
        elif self.state == "AT_DROP" and msg.data == "TIMEOUT":
            self.retry += 1
            if self.retry > self.max_retry:
                self._fail("MATCH_TIMEOUT")
            else:
                rospy.logwarn("drone %d match timeout, retry %d", self.drone_id, self.retry)
                self.state = "EXECUTE"
                self._publish_phase("EXECUTE")
                self._goto((self.drop[0], self.drop[1], self.cruise_z))
                self._pending = "DESCEND"

    def _on_payload_done(self, msg):
        if self.state == "DROP" and msg.data:
            # 返航错峰：先爬到投放点上方巡航高度悬停，drone i 延迟 i*return_stagger 后再返航
            self.return_at = rospy.get_time() + self.drone_id * self.return_stagger
            self.state = "RETURN_PENDING"
            self._publish_phase("RETURN_PENDING")
            self._goto((self.drop[0], self.drop[1], self.cruise_z))
            rospy.loginfo("drone %d return scheduled in %.1fs",
                          self.drone_id, self.drone_id * self.return_stagger)

    # ---------------------------------------------------------------- helpers
    def _compute_formation_offset(self):
        """非凸-α 编队协调层：计算本机相对共享航点的偏移。

        返回 3 元组 (dx, dy, dz) 或 None（未开启）。
        reference=pad0 时按「相对 0 号机起降点」保持初始相对位置；
        reference=explicit 时读取 formation.yaml 的 offsets[drone_id]（对齐真机 formation/drone*）。
        """
        fc = cfg.load("formation.yaml").get("formation", {})
        if not fc.get("enabled", False):
            return None
        ref = fc.get("reference", "pad0")
        if ref == "explicit":
            offs = fc.get("offsets", []) or []
            if self.drone_id < len(offs):
                o = offs[self.drone_id]
                return (float(o[0]), float(o[1]), float(o[2]))
            return (0.0, 0.0, 0.0)
        pads = self.scene.get_pads()
        p0 = pads[0] if pads else (0.0, 0.0)
        pi = pads[self.drone_id] if self.drone_id < len(pads) else (0.0, 0.0)
        return (float(pi[0]) - float(p0[0]), float(pi[1]) - float(p0[1]), 0.0)

    def _goto(self, xyz, formation=False):
        if formation and self.formation_offset is not None:
            xyz = (xyz[0] + self.formation_offset[0],
                   xyz[1] + self.formation_offset[1],
                   xyz[2] + self.formation_offset[2])
        g = PoseStamped()
        g.header.frame_id = "world"
        g.header.stamp = rospy.Time.now()
        g.pose.position.x, g.pose.position.y, g.pose.position.z = xyz
        g.pose.orientation.w = 1.0
        self.goal = xyz
        self.pub_goal.publish(g)

    def _publish_phase(self, ph):
        self.pub_phase.publish(String(data=ph))

    def _fail(self, reason):
        self.state = "FAILED"
        self._publish_phase("FAILED")
        rospy.logerr("drone %d FAILED: %s", self.drone_id, reason)

    # ---------------------------------------------------------------- tick
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._tick()
            rate.sleep()

    def _tick(self):
        if self.state == "TAKEOFF_PENDING":
            if self.takeoff_at is not None and rospy.get_time() >= self.takeoff_at:
                self.state = "TAKEOFF"
                self._publish_phase("TAKEOFF")
                self._goto((self.pad[0], self.pad[1], self.takeoff_hover_z))
            return
        if self.state == "EXECUTE_PENDING":
            if self.enter_at is not None and rospy.get_time() >= self.enter_at:
                self.state = "EXECUTE"
                self._publish_phase("EXECUTE")
                self.retry = 0
                self.crossed_zone = False
                self.pub_crossed.publish(Bool(data=False))
                # 先飞向穿越区中心（编队协调层：加每机偏移展开成队形），进入后再转向投放点
                cz = self.scene.crossing_zone_center
                self._goto((cz[0], cz[1], self.cruise_z), formation=True)
                self._pending = "CROSS_ZONE"
                rospy.loginfo("drone %d heading to crossing zone (%.1f, %.1f)",
                              self.drone_id, cz[0], cz[1])
            return
        if self.state == "RETURN_PENDING":
            if self.return_at is not None and rospy.get_time() >= self.return_at:
                self.state = "RETURN"
                self._publish_phase("RETURN")
                # 两段式返航：先以巡航高度回到降落区上空，到位再降到悬停高度。
                # 避免穿越林区时提前降到悬停高度(1.5m)在林缘死点卡住。
                self._goto((self.pad[0], self.pad[1], self.cruise_z))
                self._pending = "DESCEND_PAD"
                rospy.loginfo("drone %d return start (cruise to pad)", self.drone_id)
            return
        if self.goal is None:
            return

        # 在执行阶段检查是否进入穿越区
        if self.state == "EXECUTE":
            if not self.crossed_zone and self.scene.in_crossing_zone((self.odom[0], self.odom[1])):
                self.crossed_zone = True
                self.pub_crossed.publish(Bool(data=True))
                rospy.loginfo("drone %d crossed zone", self.drone_id)

        d = np.linalg.norm(np.array(self.goal) - np.array(self.odom))
        if self.state == "TAKEOFF" and d < self.arrival_tol:
            self.state = "TAKEOFF_DONE"
            self._publish_phase("TAKEOFF_DONE")
            rospy.loginfo("drone %d takeoff done", self.drone_id)
            self.goal = None
        elif self.state == "EXECUTE":
            if getattr(self, "_pending", None) == "CROSS_ZONE":
                # 已到达/进入穿越区后，转向投放点
                if self.crossed_zone:
                    self._pending = "DESCEND"
                    self._goto((self.drop[0], self.drop[1], self.cruise_z))
                    rospy.loginfo("drone %d crossed zone, heading to drop", self.drone_id)
                elif d < self.drop_tol:
                    # 没进入穿越区却到了中心点附近（理论不应发生），强制失败
                    self._fail("CROSSING_ZONE_MISSED")
                    return
            elif getattr(self, "_pending", None) == "DESCEND":
                if d < self.drop_tol:
                    # 强制穿越区检查
                    if not self.crossed_zone:
                        self._fail("CROSSING_ZONE_MISSED")
                        return
                    self._pending = None
                    self.state = "AT_DROP"
                    self._publish_phase("AT_DROP")
                    self.pub_at_drop.publish(Bool(data=True))
                    self._goto((self.drop[0], self.drop[1], self.identify_z))
        elif self.state == "RETURN":
            if getattr(self, "_pending", None) == "DESCEND_PAD":
                # 已到降落区上空(巡航高度)，下降到悬停高度（降落区无树，安全）
                if d < self.arrival_tol:
                    self._pending = None
                    self._goto((self.pad[0], self.pad[1], self.takeoff_hover_z))
                    rospy.loginfo("drone %d at pad, descending to hover", self.drone_id)
            elif d < self.drop_tol:
                # 已降到悬停高度并到位，任务完成
                self.state = "DONE"
                self._publish_phase("DONE")
                rospy.loginfo("drone %d mission DONE", self.drone_id)
                self.goal = None
                self.pub_at_drop.publish(Bool(data=False))


if __name__ == "__main__":
    try:
        node = MissionExecutor()
        node.run()
    except rospy.ROSInterruptException:
        pass
