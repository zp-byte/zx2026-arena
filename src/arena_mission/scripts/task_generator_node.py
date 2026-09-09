#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_mission::task_generator_node — 任务生成（全局 1 份，启动即发布，latch）。

输入: scene_topology（投送平台/起降位）+ fleet（编队）+ rules（高度/速度）
输出: /zx2026/mission/<drone_id> (Mission, latch)，每机一条。
分配规则（科目三口径）：载荷按 i%类型数 直接指派（每类 2 机），
box_color = color_map[载荷]；drop_point_id 仅作参考指派（GCS/审计/
fallback），闭环 color_id 模式下目标平台由机载识别择定，执行器不得据此导航。
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
        n_pt = len(self.scene.payload_types)
        for i in range(self.scene.drone_count):
            dp = self.scene.drop_points[i % n_dp] if n_dp else None
            m = Mission()
            m.drone_id = i
            # 载荷按 i%类型数 直接指派（每类 2 机），不再从投放点反推
            m.payload_type = self.scene.payload_types[i % n_pt] if n_pt else "TYPE_A"
            m.drop_point_id = dp.id if dp else -1      # 参考指派（审计/fallback）
            m.marker_id = dp.marker if dp else ""
            m.box_color = self.scene.color_for_type(m.payload_type)
            m.cruise_z = self.cruise_z
            m.max_vel = self.max_vel
            m.mission_seq = i
            if i < len(pads):
                m.takeoff_pose.position = Point(pads[i][0], pads[i][1], self.takeoff_z)
            else:
                m.takeoff_pose.position = Point(0.0, 0.0, self.takeoff_z)
            m.takeoff_pose.orientation = Quaternion(w=1.0)
            self.pubs[i].publish(m)
            rospy.loginfo("  mission[%d]: drone%d box=%s(%s) ref_dp%d(%s)",
                          i, i, m.payload_type, m.box_color,
                          m.drop_point_id, m.marker_id)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = TaskGenerator()
        node.run()
    except rospy.ROSInterruptException:
        pass
