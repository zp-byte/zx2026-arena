#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_sensor::tag_detector_node — 投放点 Tag 检测（每机一个）。

输入: /drone_<id>/odom + /zx2026/scene
输出: /drone_<id>/detected/tag_id, /drone_<id>/detected/type,
      /drone_<id>/detected/color, /drone_<id>/detected/confidence
判定: 距离 ≤ range ∧ 在 FOV 内 ∧ 无遮挡（几何三重判定，区别于旧 sim_detector 的
       "距投放点<5m 即命中"）。
诚实性: 检测输出的是"可见平台"的真值颜色编码（无相机渲染，可见性由几何
       三重判定门控）；color_id 闭环下"选哪个平台"的决策仍完全在机载。
"""
import math
import random

import rospy
from std_msgs.msg import Int32, UInt8, Float32
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

from zx2026_common import config as cfg
from zx2026_common import scene as sc
from zx2026_common.scene import Scene
from arena_sensor.sensor_common import load_tag_params, detect_tag


class TagDetectorNode:
    def __init__(self):
        rospy.init_node("tag_detector_node", anonymous=True)
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id

        # 相机模式自门控（2026-09-10 相机立项）：color_id.source=camera 时真值
        # 检测器让位——在创建任何 Publisher 前退出，结构上消灭 /detected/*
        # 双发布者竞态（camera_sim_node/color_detector_node 同款自门控）。
        source = (cfg.load("competition_rules.yaml").get("color_id", {})
                  .get("source", "truth"))
        if source == "camera":
            rospy.loginfo("tag_detector_node: disabled (color_id.source=camera)")
            return

        self.scene = Scene()
        self.params = load_tag_params()
        self.rng = random.Random(self.params["rng_seed"] + self.drone_id * 7919)
        self.odom_pos = (0.0, 0.0, 1.0)
        self.odom_yaw = 0.0

        self.pub_tag = rospy.Publisher(ns + "/detected/tag_id", Int32, queue_size=10)
        self.pub_type = rospy.Publisher(ns + "/detected/type", UInt8, queue_size=10)
        self.pub_color = rospy.Publisher(ns + "/detected/color", UInt8, queue_size=10)
        self.pub_conf = rospy.Publisher(ns + "/detected/confidence", Float32, queue_size=10)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.Subscriber("/zx2026/scene", PoseArray, self._on_scene)
        rospy.loginfo("tag_detector_node: drone %d (range=%.1f fov=%.0f)",
                      self.drone_id, self.params["range"], self.params["fov_deg"])

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_scene(self, msg):
        pass  # 场景由 Scene() 静态构建，PoseArray 仅作同步参考

    def run(self):
        if getattr(self, "scene", None) is None:   # 自门控早退（source=camera）
            return
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._publish_detection()
            rate.sleep()

    def _publish_detection(self):
        dp, conf = detect_tag(self.scene, self.odom_pos, self.odom_yaw, self.params, rng=self.rng)
        if dp is None:
            self.pub_tag.publish(Int32(data=-1))
            self.pub_type.publish(UInt8(data=255))
            self.pub_color.publish(UInt8(data=255))
            self.pub_conf.publish(Float32(data=0.0))
            return
        self.pub_tag.publish(Int32(data=dp.id))
        self.pub_type.publish(UInt8(data=cfg.type_to_uint8(dp.type_id)))
        self.pub_color.publish(UInt8(data=cfg.color_to_uint8(dp.color)))
        self.pub_conf.publish(Float32(data=conf))
        rospy.logdebug_throttle(2.0,
                                "drone %d sees drop point %d (%s) conf=%.2f",
                                self.drone_id, dp.id, dp.type_id, conf)


if __name__ == "__main__":
    try:
        node = TagDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
