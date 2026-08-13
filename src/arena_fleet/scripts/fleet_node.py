#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_fleet::fleet_node — 编队就绪监控。

订阅全部 /drone_<id>/odom，确认所有 6 机 odom 到齐后发布
/zx2026/fleet_ready (Bool)，并发布起降位姿数组 /zx2026/pads (PoseArray)。
"""
import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion

from zx2026_common import config as cfg
from zx2026_common.scene import Scene


class FleetNode:
    def __init__(self):
        rospy.init_node("fleet_node", anonymous=True)
        self.scene = Scene()
        self.expected = self.scene.drone_count
        self.odom_ok = {}
        self.pub_ready = rospy.Publisher("/zx2026/fleet_ready", Bool, queue_size=1, latch=True)
        self.pub_pads = rospy.Publisher("/zx2026/pads", PoseArray, queue_size=1, latch=True)
        for i in range(self.expected):
            rospy.Subscriber("/drone_%d/odom" % i, Odometry,
                             lambda msg, i=i: self._on_odom(i, msg))
        self._publish_pads()
        rospy.loginfo("fleet_node: expecting %d drones", self.expected)

    def _on_odom(self, i, msg):
        self.odom_ok[i] = True
        if len(self.odom_ok) == self.expected:
            self.pub_ready.publish(Bool(data=True))
            if not getattr(self, "_ready_logged", False):
                self._ready_logged = True
                rospy.loginfo("fleet_node: all %d odoms present, fleet ready", self.expected)

    def _publish_pads(self):
        pa = PoseArray()
        pa.header.frame_id = "world"
        for (x, y) in self.scene.get_pads():
            p = Pose()
            p.position = Point(x, y, self.scene.venue["ground_z"] + 0.01)
            p.orientation.w = 1.0
            pa.poses.append(p)
        self.pub_pads.publish(pa)


if __name__ == "__main__":
    try:
        node = FleetNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
