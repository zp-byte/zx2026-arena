#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_sensor::lidar_node — 模拟 3D 激光点云（每机一个）。

输入: /drone_<id>/odom
输出: /drone_<id>/cloud (sensor_msgs/PointCloud2, xyz, 世界系)
实现: 对 scene 障碍做确定性 raycast，含 seed 化测距噪声。
用途: arena_nav 构建局部占用栅格；亦可接外部感知算法。
"""
import math
import struct
import random

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField

from zx2026_common import config as cfg
from zx2026_common import geometry as geo
from zx2026_common.scene import Scene


class LidarNode:
    def __init__(self):
        rospy.init_node("lidar_node", anonymous=True)
        self.drone_id = int(rospy.get_param("~drone_id", 0))
        ns = "/drone_%d" % self.drone_id

        settings = cfg.load("sim_settings.yaml")
        self.seed = int(settings.get("run_seed", 42))
        self.scene = Scene()
        drone_cfg = self.scene.drone_config(self.drone_id)
        self.range = drone_cfg.sensor_range if drone_cfg else 5.0

        # ray 模式：方位角 × 俯仰角
        self.az_steps = 72       # 每圈
        cl = settings.get("closed_loop", {})
        if cl.get("lidar_top_blind", False):
            # Mid-360 式顶盲锥：俯仰约 -40° ~ +20°，正上方留盲区
            self.el_vals = [-0.70, -0.45, -0.25, -0.10, 0.0, 0.20, 0.35]
        else:
            self.el_vals = [-0.5, -0.25, 0.0, 0.25, 0.5]
        # 传感器噪声模型（物理正确：角度噪声注入到射线方向，距离噪声沿射线方向）
        sl = settings.get("sensor", {}).get("lidar", {})
        self.range_noise_std = float(sl.get("range_noise_std", 0.02))
        self.dropout_rate = float(sl.get("dropout_rate", 0.05))
        self.angular_noise_rad = math.radians(float(sl.get("angular_noise_deg", 0.5)))
        self.false_positive_rate = float(sl.get("false_positive_rate", 0.0))
        self.rng = random.Random(self.seed + self.drone_id * 7919)

        self.odom_pos = (0.0, 0.0, 1.0)
        self.odom_yaw = 0.0
        self.pub = rospy.Publisher(ns + "/cloud", PointCloud2, queue_size=4)
        rospy.Subscriber(ns + "/odom", Odometry, self._on_odom)
        rospy.loginfo("lidar_node: drone %d range=%.1f",
                      self.drone_id, self.range)

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_pos = (p.x, p.y, p.z)
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self._publish()
            rate.sleep()

    def _publish(self):
        pts = []
        origin = self.odom_pos
        for az_i in range(self.az_steps):
            az = self.odom_yaw + az_i * (2.0 * math.pi / self.az_steps)
            for el in self.el_vals:
                # 角度噪声：射线方向微偏（物理正确的噪声注入点）
                if self.angular_noise_rad > 0:
                    az_noisy = az + self.rng.gauss(0.0, self.angular_noise_rad)
                    el_noisy = el + self.rng.gauss(0.0, self.angular_noise_rad)
                else:
                    az_noisy, el_noisy = az, el
                dx = math.cos(el_noisy) * math.cos(az_noisy)
                dy = math.cos(el_noisy) * math.sin(az_noisy)
                dz = math.sin(el_noisy)
                d = geo.normalize((dx, dy, dz))
                t = self.scene.raycast(origin, d, max_t=self.range)
                if t is None:
                    continue
                # 随机丢失
                if self.dropout_rate > 0 and self.rng.random() < self.dropout_rate:
                    continue
                # 距离噪声（与距离成正比，沿射线方向）
                noisy_t = t + self.rng.gauss(0.0, self.range_noise_std * t)
                if noisy_t < 0.0:
                    continue
                hit = geo.add(origin, geo.scale(d, noisy_t))
                pts.append(hit)
        # 随机误检（默认关，真机 lidar 误检率低）
        if self.false_positive_rate > 0:
            n_fp = max(1, int(self.rng.random() * self.false_positive_rate *
                              self.az_steps * len(self.el_vals)))
            for _ in range(n_fp):
                fp_az = self.rng.uniform(0, 2.0 * math.pi)
                fp_el = self.rng.uniform(-0.7, 0.35)
                fp_r = self.rng.uniform(0.1, self.range)
                fp_dx = math.cos(fp_el) * math.cos(fp_az)
                fp_dy = math.cos(fp_el) * math.sin(fp_az)
                fp_dz = math.sin(fp_el)
                fp_dir = geo.normalize((fp_dx, fp_dy, fp_dz))
                fp_hit = geo.add(origin, geo.scale(fp_dir, fp_r))
                pts.append(fp_hit)

        pc = PointCloud2()
        pc.header.stamp = rospy.Time.now()
        pc.header.frame_id = "world"
        pc.height = 1
        pc.width = len(pts)
        pc.is_dense = True
        pc.point_step = 12        # x/y/z 三个 FLOAT32，无填充
        pc.row_step = 12 * len(pts)
        pc.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        buf = []
        for (x, y, z) in pts:
            buf.append(struct.pack("<fff", float(x), float(y), float(z)))
        pc.data = b"".join(buf)
        self.pub.publish(pc)


if __name__ == "__main__":
    try:
        node = LidarNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
