#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""临时诊断：群集涌现行为检查 —— 全 6 机 odom 采样 + 群指标。

随 zx2026_all.launch 启动，等 /zx2026/start 后每 1s 采样 6 机 (x,y,vx,vy)，
输出每样本：最小机间距离（分离）、群直径（聚合）、速度朝向角差 std（对齐）。
用法: bash -c 'source devel/setup.bash && python3 flock_obs.py'
"""
import math
import time

import rospy
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped


class FlockObs:
    def __init__(self):
        rospy.init_node("flock_obs", anonymous=True)  # 先 init，避免回调竞态
        self.state = "?"
        self.odom = {}
        self.phases = {}
        self.match = {}
        self.score_summary = None
        self.fleet_ready = False
        for i in range(6):
            rospy.Subscriber("/drone_%d/odom" % i, Odometry,
                             lambda m, i=i: self._on_odom(i, m))
            rospy.Subscriber("/drone_%d/mission/phase" % i, String,
                             lambda m, i=i: self._on_phase(i, m))
            rospy.Subscriber("/drone_%d/match/result" % i, String,
                             lambda m, i=i: self._on_match(i, m))
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.Subscriber("/zx2026/score_summary", String, self._on_score)
        rospy.Subscriber("/zx2026/fleet_ready", Bool, self._on_fleet)
        # 通信观测（drone_0 视角）：中继话题的延迟/丢包（comms 开启才有消息）
        # 观测窗口 450s（R1 调参：风场使任务 ~2 倍慢，300s 窗口截断了返航腿）
        self.comm = {}        # j -> (last_stamp, last_seq, latency)
        self.comm_gaps = {}   # j -> 累计 stamp 间隔间隙（丢包代理）
        self.comm_lat_sum = {}  # j -> (sum_lat, count) 最终平均延迟
        for j in range(6):
            if j == 0:
                continue
            rospy.Subscriber("/drone_0/odom_from_%d" % j, Odometry,
                             lambda m, j=j: self._on_comm(j, m))
        self.wind = None
        rospy.Subscriber("/zx2026/wind", Vector3Stamped, self._on_wind)

    def _on_odom(self, i, m):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        self.odom[i] = ((p.x, p.y, p.z), (v.x, v.y, v.z))

    def _on_phase(self, i, m):
        self.phases[i] = m.data

    def _on_match(self, i, m):
        self.match[i] = m.data

    def _on_score(self, m):
        self.score_summary = m.data

    def _on_state(self, m):
        self.state = m.data

    def _on_fleet(self, m):
        self.fleet_ready = m.data

    def _on_comm(self, j, m):
        now = rospy.get_time()
        lat = now - m.header.stamp.to_sec()
        prev = self.comm.get(j)
        if prev is not None:
            # seq 不可靠（rospy 重序列化会重发 header.seq），改按 header.stamp 间隔：
            # 名义发布周期 50ms，间隔超过 ~1.5 周期视为中间有丢包
            dt = m.header.stamp.to_sec() - prev[0]
            if dt > 0.075:
                self.comm_gaps[j] = self.comm_gaps.get(j, 0) + 1
        self.comm[j] = (m.header.stamp.to_sec(), m.header.seq, lat)
        sl, c = self.comm_lat_sum.get(j, (0.0, 0))
        self.comm_lat_sum[j] = (sl + lat, c + 1)

    def _on_wind(self, m):
        self.wind = (m.vector.x, m.vector.y)

    def run(self):
        t0 = time.time()
        while not self.fleet_ready and time.time() - t0 < 30:
            rospy.sleep(0.2)
        t0 = time.time()
        while self.state != "P2_WAIT" and time.time() - t0 < 20:
            rospy.sleep(0.2)
        srv = rospy.ServiceProxy("/zx2026/start", Trigger)
        srv()
        print("== flock obs started ==")
        t0 = time.time()
        last_state = None
        while time.time() - t0 < 450:
            if self.state != last_state:
                last_state = self.state
                print("-- state ->", self.state, "@ %.0fs" % (time.time() - t0))
            if self.state == "DONE":
                rospy.sleep(1.0)
                break
            o = [self.odom.get(i) for i in range(6)]
            t = time.time() - t0
            if all(x is not None for x in o):
                pos = [x[0] for x in o]
                vel = [x[1] for x in o]
                dmin = min(math.hypot(pos[i][0] - pos[j][0], pos[i][1] - pos[j][1])
                           for i in range(6) for j in range(i + 1, 6))
                cx = sum(p[0] for p in pos) / 6.0
                cy = sum(p[1] for p in pos) / 6.0
                spread = max(math.hypot(p[0] - cx, p[1] - cy) for p in pos)
                heads = [math.atan2(v[1], v[0]) for v in vel]
                hmean = math.atan2(sum(math.sin(h) for h in heads),
                                   sum(math.cos(h) for h in heads))
                hdiff = [abs((h - hmean + math.pi) % (2 * math.pi) - math.pi)
                         for h in heads]
                print("t=%5.1f dmin=%4.2f spread=%4.2f headdiff_max=%4.2f z=%s"
                      % (t, dmin, spread, max(hdiff),
                         [round(p[2], 1) for p in pos]))
            if self.comm:
                cl = [round(self.comm[j][2], 3) for j in sorted(self.comm)]
                cg = sum(self.comm_gaps.values())
                print("  comm_lat(0<-j)=%s gaps_total=%d wind=%s"
                      % (cl, cg,
                         None if self.wind is None
                         else (round(self.wind[0], 3), round(self.wind[1], 3))))
            rospy.sleep(1.0)
        print("== flock obs done ==")
        print("final state:", self.state)
        print("phases:", {k: v for k, v in sorted(self.phases.items())})
        print("match:", {k: v for k, v in sorted(self.match.items())})
        print("score_summary:", self.score_summary)
        print("comm_gaps:", self.comm_gaps)
        print("comm_mean_lat:", {j: round(s / c, 3) if c else None
                                 for j, (s, c) in sorted(self.comm_lat_sum.items())})
        print("wind:", self.wind)


if __name__ == "__main__":
    try:
        FlockObs().run()
    except rospy.ROSInterruptException:
        pass
