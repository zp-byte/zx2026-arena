#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_sensor::tag_detector_node — 投放点 Tag 检测（每机一个）。

输入: /drone_<id>/odom + /zx2026/scene
输出: /drone_<id>/detected/tag_id, /drone_<id>/detected/type, /drone_<id>/detected/confidence
判定: 距离 ≤ range ∧ 在 FOV 内 ∧ 无遮挡（几何三重判定，区别于旧 sim_detector 的
       "距投放点<5m 即命中"）。
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

        self.scene = Scene()
        self.params = load_tag_params()
        self.rng = random.Random(self.params["rng_seed"] + self.drone_id * 7919)
        self.odom_pos = (0.0, 0.0, 1.0)
        self.odom_yaw = 0.0

        self.pub_tag = rospy.Publisher(ns + "/detected/tag_id", Int32, queue_size=10)
        self.pub_type = rospy.Publisher(ns + "/detected/type", UInt8, queue_size=10)
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
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self._publish_detection()
            rate.sleep()

    def _publish_detection(self):
        dp, conf = detect_tag(self.scene, self.odom_pos, self.odom_yaw, self.params, rng=self.rng)
        if dp is None:
            self.pub_tag.publish(Int32(data=-1))
            self.pub_type.publish(UInt8(data=255))
            self.pub_conf.publish(Float32(data=0.0))
            return
        self.pub_tag.publish(Int32(data=dp.id))
        self.pub_type.publish(UInt8(data=cfg.type_to_uint8(dp.type_id)))
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
