#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_mission::task_generator_node — 任务生成（全局 1 份，启动即发布，latch）。

输入: scene_topology（投放点/起降位）+ fleet（编队）+ rules（高度/速度）
输出: /zx2026/mission/<drone_id> (Mission, latch)，每机一条。
分配规则：每机一个投放点（多于投放点时轮转），本机 payload_type = 投放点 type_id
（即"携带与投放点匹配的物品"）；随机性来自 run_seed（用于投放点轮转/排序种子）。
"""
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String

from zx2026_common import config as cfg
from zx2026_common.scene import Scene
from zx2026_common.msg import Mission


class TaskGenerator:
    def __init__(self):
        rospy.init_node("task_generator_node", anonymous=True)
        self.scene = Scene()
        rules = cfg.load("competition_rules.yaml")
        self.cruise_z = float(rules["heights"].get("cruise_z", 2.5))
        self.max_vel = float(rules["motion"].get("default_max_vel", 2.0))
        self.takeoff_z = float(rules["heights"].get("takeoff_hover_z", 1.5))

        self.pubs = {}
        for i in range(self.scene.drone_count):
            self.pubs[i] = rospy.Publisher("/zx2026/mission/%d" % i, Mission,
                                           queue_size=1, latch=True)

        self._generate_and_publish()
        rospy.loginfo("task_generator_node: %d missions published", self.scene.drone_count)

    def _generate_and_publish(self):
        pads = self.scene.get_pads()
        n_dp = len(self.scene.drop_points)
        for i in range(self.scene.drone_count):
            dp = self.scene.drop_points[i % n_dp] if n_dp else None
            m = Mission()
            m.drone_id = i
            m.payload_type = dp.type_id if dp else "TYPE_A"
            m.drop_point_id = dp.id if dp else -1
            m.marker_id = dp.marker if dp else ""
            m.cruise_z = self.cruise_z
            m.max_vel = self.max_vel
            m.mission_seq = i
            if i < len(pads):
                m.takeoff_pose.position = Point(pads[i][0], pads[i][1], self.takeoff_z)
            else:
                m.takeoff_pose.position = Point(0.0, 0.0, self.takeoff_z)
            m.takeoff_pose.orientation = Quaternion(w=1.0)
            self.pubs[i].publish(m)
            rospy.loginfo("  mission[%d]: drone%d → drop_point%d (%s)",
                          i, i, m.drop_point_id, m.payload_type)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = TaskGenerator()
        node.run()
    except rospy.ROSInterruptException:
        pass
