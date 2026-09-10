#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_sensor::camera_sim_node — 下视相机合成图像源（每机一个）。

color_id.source=camera 的图像源：从 Scene 真值几何 + 真值 odom 渲染
sensor_msgs/Image（camera_render.render_frame，传感器模型哲学同 lidar_node）。
发布 /drone_<id>/camera/image_raw (bgr8) + /drone_<id>/camera/camera_info。

自门控（诚实边界）：color_id.source=truth（回退显式）时本节点启动即退、
不创建任何 Publisher——与真值 tag_detector 结构上互斥，消灭 /detected/*
双发布者竞态。整帧丢失（dropout）时本帧不发布，由检测器看门狗发哨兵。
"""
import math
import random

import numpy as np

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray

from zx2026_common import config as cfg
from zx2026_common.scene import Scene
from arena_sensor.camera_render import render_frame, make_cam_cfg, intrinsics


class CameraSimNode:
    def __init__(self):
        rospy.init_node("camera_sim_node", anonymous=True)
        self._active = False
        source = (cfg.load("competition_rules.yaml").get("color_id", {})
                  .get("source", "camera"))
        if source != "camera":
            rospy.loginfo("camera_sim_node: disabled (color_id.source=%s)", source)
            return
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id

        settings = cfg.load("sim_settings.yaml")
        self.scene = Scene()
        self.cam_cfg = make_cam_cfg(settings.get("sensor", {}).get("camera", {}))
        self.fps = float(settings.get("sensor", {}).get("camera", {}).get("fps", 12))
        self.rng = random.Random(int(settings.get("run_seed", 42))
                                 + self.drone_id * 7919)

        self.odom_pos = (0.0, 0.0, 1.0)
        self.odom_yaw = 0.0

        self.pub_img = rospy.Publisher(ns + "/camera/image_raw", Image, queue_size=2)
        self.pub_info = rospy.Publisher(ns + "/camera/camera_info", CameraInfo,
                                        queue_size=1, latch=True)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.loginfo("camera_sim_node: drone %d (%dx%d@%.0ffps vfov=%.0f)",
                      self.drone_id, self.cam_cfg["width"], self.cam_cfg["height"],
                      self.fps, self.cam_cfg["vfov_deg"])
        self._active = True

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def run(self):
        if not self._active:
            return
        self._publish_info()
        rate = rospy.Rate(self.fps)
        while not rospy.is_shutdown():
            img = render_frame(self.scene, self.odom_pos, self.odom_yaw,
                               self.cam_cfg, self.rng)
            if img is not None:
                self._publish_img(img)
            rate.sleep()

    def _publish_img(self, img):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.height, msg.width = img.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = img.tobytes()
        self.pub_img.publish(msg)

    def _publish_info(self):
        fx, fy, cx, cy = intrinsics(self.cam_cfg)
        info = CameraInfo()
        info.header.frame_id = "world"
        info.width = self.cam_cfg["width"]
        info.height = self.cam_cfg["height"]
        info.distortion_model = "plumb_bob"
        info.D = Float64MultiArray(data=[0.0] * 5)   # 大写：精简版消息无小写畸变字段
        info.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)


if __name__ == "__main__":
    try:
        node = CameraSimNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
