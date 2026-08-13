#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_world::scene_marker_node — 仅发布静态场景（投放点 PoseArray + RViz 森林 MarkerArray）。

用于 Gazebo 物理后端：world_node 不运行（odom/碰撞由 gzserver 插件负责），
但 RViz 仍需要森林/区域/投放点标记。数据来源与 python 后端完全一致
（同一个 Scene，同 seed），保证两种后端视图一致。
"""
import rospy
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray

from zx2026_common import scene as sc
from arena_world.scene_markers import build_scene_messages


def main():
    rospy.init_node("scene_marker_node", anonymous=False)
    scene = sc.Scene()
    pub_scene = rospy.Publisher("/zx2026/scene", PoseArray, queue_size=1, latch=True)
    pub_markers = rospy.Publisher("/zx2026/markers", MarkerArray, queue_size=1, latch=True)

    pa, ma = build_scene_messages(scene)
    rospy.loginfo("scene_marker_node: %d drop points, %d markers",
                  len(pa.poses), len(ma.markers))

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pa.header.stamp = rospy.Time.now()
        pub_scene.publish(pa)
        pub_markers.publish(ma)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
