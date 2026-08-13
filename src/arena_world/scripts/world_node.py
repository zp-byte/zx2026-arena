#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_world::world_node — 科目三 主仿真循环。

职责：
  * 解析 scene_topology/fleet/sim_settings，构建 Scene；
  * 在起降区 pads 生成 6 架 DroneSim（物理后端可切换）；
  * 订阅 /drone_<id>/vel_cmd，按 control_dt 步进全部动力学；
  * 每步碰撞检测（障碍 + 地面 + 六机互撞）；
  * 发布 /clock（sim time）、/drone_<id>/odom、
    /zx2026/scene（投放点 PoseArray）、/zx2026/markers（RViz）；
  * 服务 /zx2026/world/reset（重置到初始位姿）。
"""
import math

import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseArray, Pose, Point, Quaternion, Vector3, TransformStamped
from tf.msg import tfMessage
from visualization_msgs.msg import Marker, MarkerArray

from zx2026_common import config as cfg
from zx2026_common import scene as sc
from zx2026_common import geometry as geo
from arena_world.dynamics import make_backend, DroneState
from arena_world.scene_markers import build_scene_messages


class DroneSim:
    def __init__(self, drone_id, pose, backend_name, settings):
        self.drone_id = drone_id
        self.backend = make_backend(backend_name, settings)
        self.backend.reset(pose)
        self.home = list(pose[:3])
        self.cmd_vel = np.zeros(3)
        self.cmd_yaw_rate = 0.0
        self.collided = False
        self._collide_logged = False

    def set_vel_cmd(self, vx, vy, vz, yaw_rate):
        self.cmd_vel = np.array([vx, vy, vz])
        self.cmd_yaw_rate = yaw_rate

    def step(self, dt):
        self.backend.set_vel_cmd(self.cmd_vel, self.cmd_yaw_rate)
        self.backend.step(dt)

    def state(self):
        return self.backend.get_state()


class WorldNode:
    def __init__(self):
        rospy.init_node("world_node", anonymous=False)

        self.sim_settings = cfg.load("sim_settings.yaml")
        self.scene = sc.Scene()

        self.use_sim_time = bool(self.sim_settings.get("use_sim_time", True))
        self.world_dt = float(self.sim_settings.get("world_dt", 0.01))
        self.control_dt = float(self.sim_settings.get("control_dt", 0.05))
        self.publish_dt = float(self.sim_settings.get("publish_dt", 0.05))
        self.backend_name = self.sim_settings.get("backend", "cascade_pid")

        # 起降区 pads → 6 机初始位姿
        pads = self.scene.get_pads()
        self.drones = {}
        for i, (px, py) in enumerate(pads):
            if i >= self.scene.drone_count:
                break
            dz = float(self.scene.venue["ground_z"]) + 1.0
            pose = (px, py, dz, 0.0)
            self.drones[i] = DroneSim(i, pose, self.backend_name, self.sim_settings)

        # ---- 话题 ----
        self.pub_clock = rospy.Publisher("/clock", Clock, queue_size=1, latch=True)
        self.pub_scene = rospy.Publisher("/zx2026/scene", PoseArray, queue_size=1, latch=True)
        self.pub_markers = rospy.Publisher("/zx2026/markers", MarkerArray, queue_size=1, latch=True)
        self.pub_odom = {}
        self.pub_collision = {}
        self.sub_vel = {}
        for i, d in self.drones.items():
            ns = "/drone_%d" % i
            self.pub_odom[i] = rospy.Publisher(ns + "/odom", Odometry, queue_size=10)
            self.pub_collision[i] = rospy.Publisher(ns + "/collision", Bool, queue_size=1, latch=True)
            self.sub_vel[i] = rospy.Subscriber(
                ns + "/vel_cmd", Twist, lambda msg, i=i: self._on_vel_cmd(i, msg))
        # TF：world → drone_<i>，供 RViz 显示无人机位姿/轨迹
        self.tf_pub = rospy.Publisher("/tf", tfMessage, queue_size=10)

        # ---- 服务 ----
        self.srv_reset = rospy.Service("/zx2026/world/reset", Empty, self._on_reset)

        # 模拟时钟
        self.sim_t = 0.0
        self._acc_control = 0.0
        self._acc_pub = 0.0

        rospy.loginfo("world_node: %d drones, backend=%s, dt=%s",
                      len(self.drones), self.backend_name, self.world_dt)
        self._publish_scene_static()

    # ---------------------------------------------------------------- callbacks
    def _on_vel_cmd(self, i, msg):
        d = self.drones[i]
        d.set_vel_cmd(msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.z)

    def _on_reset(self, req):
        for i, d in self.drones.items():
            d.backend.reset((d.home[0], d.home[1], d.home[2], 0.0))
            d.collided = False
            d.cmd_vel = np.zeros(3)
        self.sim_t = 0.0
        return EmptyResponse()

    # ---------------------------------------------------------------- main loop
    def run(self):
        rate = rospy.Rate(1.0 / self.world_dt)
        while not rospy.is_shutdown():
            self.step_once()
            rate.sleep()

    def step_once(self):
        dt = self.world_dt
        self.sim_t += dt

        # 碰撞检测（对上一状态）：障碍 + 地面
        for i, d in self.drones.items():
            p = d.state().pos_tuple()
            if self.scene.collides(p):
                d.collided = True
                self.pub_collision[i].publish(Bool(data=True))
                if not d._collide_logged:
                    d._collide_logged = True
                    rospy.logwarn("drone %d COLLIDED obstacle at (%.2f,%.2f,%.2f)",
                                  i, p[0], p[1], p[2])

        # 六机互撞
        states = {i: d.state() for i, d in self.drones.items()}
        for i, si in states.items():
            if self.drones[i].collided:
                continue
            for j, sj in states.items():
                if i >= j:
                    continue
                if geo.norm2((si.pos[0] - sj.pos[0], si.pos[1] - sj.pos[1],
                              si.pos[2] - sj.pos[2])) <= (2 * self.scene.drone_radius) ** 2:
                    self.drones[i].collided = True
                    self.drones[j].collided = True
                    self.pub_collision[i].publish(Bool(data=True))
                    self.pub_collision[j].publish(Bool(data=True))
                    for k in (i, j):
                        if not self.drones[k]._collide_logged:
                            self.drones[k]._collide_logged = True
                            q = self.drones[k].state().pos_tuple()
                            rospy.logwarn("drone %d COLLIDED inter-drone with %d at (%.2f,%.2f,%.2f)",
                                          k, (j if k == i else i), q[0], q[1], q[2])

        # 步进
        for d in self.drones.values():
            if d.collided:
                # 碰撞后冻结，避免穿透
                d.cmd_vel = np.zeros(3)
            d.step(dt)

        # 时钟发布（每 control tick 同步一次 /clock）
        self._acc_pub += dt
        if self._acc_pub >= self.publish_dt:
            self._acc_pub = 0.0
            self._publish_clock_and_odom()

    # ---------------------------------------------------------------- publishing
    def _publish_clock_and_odom(self):
        if self.use_sim_time:
            c = Clock()
            c.clock.secs = int(self.sim_t)
            c.clock.nsecs = int((self.sim_t - int(self.sim_t)) * 1e9)
            self.pub_clock.publish(c)
        for i, d in self.drones.items():
            od = Odometry()
            od.header.stamp = rospy.Time.now()
            od.header.frame_id = "world"
            od.child_frame_id = "drone_%d" % i
            s = d.state()
            od.pose.pose.position = Point(s.pos[0], s.pos[1], s.pos[2])
            od.pose.pose.orientation = quat_from_yaw(s.yaw)
            od.twist.twist.linear.x = s.vel[0]
            od.twist.twist.linear.y = s.vel[1]
            od.twist.twist.linear.z = s.vel[2]
            self.pub_odom[i].publish(od)
            # 世界系→机体 TF（位姿 + yaw）
            tr = TransformStamped()
            tr.header.stamp = od.header.stamp
            tr.header.frame_id = "world"
            tr.child_frame_id = "drone_%d" % i
            tr.transform.translation = Vector3(s.pos[0], s.pos[1], s.pos[2])
            tr.transform.rotation = quat_from_yaw(s.yaw)
            self.tf_pub.publish(tfMessage(transforms=[tr]))

    def _publish_scene_static(self):
        pa, ma = build_scene_messages(self.scene)
        pa.header.stamp = rospy.Time.now()
        self.pub_scene.publish(pa)
        self.pub_markers.publish(ma)
        rospy.loginfo("world_node: scene published (%d drop points, %d markers)",
                      len(self.scene.drop_points), len(ma.markers))


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


if __name__ == "__main__":
    try:
        node = WorldNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
