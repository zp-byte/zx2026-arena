#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_mission::drop_simulator_node — 投放模拟（全局 1 份）。

订阅 6 机 /drone_<id>/payload/command (UInt8 类型)，延迟 0.5s 后回
/drone_<id>/payload/done (Bool)。记录投放类型供 scorekeeper 参考。
"""
import threading

import rospy
from std_msgs.msg import UInt8, Bool

from zx2026_common import config as cfg


class DropSimulatorNode:
    def __init__(self):
        rospy.init_node("drop_simulator_node", anonymous=True)
        self.scene_cfg = cfg.load("scene_topology.yaml")
        n = len(cfg.load("fleet.yaml").get("drones", []))
        self.pub = {}
        self.sub = {}
        self.dropped = {}   # drone_id -> (type_uint8, ok)
        for i in range(n):
            ns = "/drone_%d" % i
            self.pub[i] = rospy.Publisher(ns + "/payload/done", Bool, queue_size=10)
            self.sub[i] = rospy.Subscriber(ns + "/payload/command", UInt8,
                                           lambda m, i=i: self._on_cmd(i, m))
        rospy.loginfo("drop_simulator_node: %d payload channels", n)

    def _on_cmd(self, i, msg):
        t = cfg.uint8_to_type(msg.data)
        rospy.loginfo("drop_simulator: drone %d releasing type %s", i, t)
        self.dropped[i] = (msg.data, True)
        # 0.5s 后确认
        threading.Timer(0.5, lambda: self.pub[i].publish(Bool(data=True))).start()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = DropSimulatorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
