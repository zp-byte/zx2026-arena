#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_fleet::comm_model_node — 通信模型：邻居信息无线链路（延迟+抖动+丢包+突发丢失）。

输入: /drone_<i>/odom（world_node 真值，20Hz）
输出: /drone_<r>/odom_from_<s>（Odometry 原样透传，保留 header.stamp 供观测）

模型（每条有序链路 s→r, s≠r 独立，rng 各自派生）:
  * 延迟：入队 release_time = now + latency + uniform(0, jitter)，单调递增保证
    同链路 FIFO 不超车；每 tick 释放到期消息时发布队列【最老】 q[0]（正是
    latency 前发出的那条 → 接收方看到的数据恰为配置延迟），队列有界。
    注：发布"最新"（newest-wins）会把有效延迟塌缩成 ~1 帧周期（R2 实测 ~10ms
    而非 100ms），故弃用——真实链路语义是"发什么收什么、延迟送达"。
  * 丢包：输入侧判定（突发剩余>0 或 rng<dropout_rate 则丢）——断连期间无过期
    消息残留重放。
  * 突发：burst_prob 每链路每秒起始概率，突发期间连续丢 burst_len 条。

失联语义由 nav_node 侧实现（超过 expire_s 未更新 → 移除邻居，"感知不到的避不开"）。
本机 odom 不经过模型（自身传感器，定位不确定性由 closed_loop.drift 覆盖）。
comms.enabled=false 时本节点空闲（零发布），nav 保持直连订阅 → 零侵入。
"""
import random
from collections import deque

import rospy
from nav_msgs.msg import Odometry

from zx2026_common import config as cfg


class CommModelNode:
    def __init__(self):
        rospy.init_node("comm_model", anonymous=False)
        s = cfg.load("sim_settings.yaml")
        cm = s.get("comms", {})
        if not bool(cm.get("enabled", False)):
            rospy.loginfo("comm_model: disabled -> idle (zero pubs)")
            return
        self.latency = float(cm.get("latency_ms", 100)) / 1000.0
        self.jitter = float(cm.get("latency_jitter_ms", 40)) / 1000.0
        self.dropout = float(cm.get("dropout_rate", 0.02))
        self.burst_prob = float(cm.get("burst_prob", 0.002))
        bl = cm.get("burst_len", [3, 10])
        self.burst_len = (int(bl[0]), int(bl[1]))
        seed = int(s.get("run_seed", 42))
        self.n = int(cfg.load("fleet.yaml").get("drone_count", 6))

        # 每条链路独立缓冲 + RNG（固定插入顺序 → 固定 tick 遍历顺序，近可复现）
        self.links = {}
        self.pubs = {}
        for r in range(self.n):
            for sr in range(self.n):
                if sr == r:
                    continue
                self.links[(sr, r)] = {
                    "queue": deque(),
                    "burst": 0,
                    "rng": random.Random(seed + 7919 * sr + 104729 * r),
                    "prev": 0.0,
                }
                self.pubs[(sr, r)] = rospy.Publisher(
                    "/drone_%d/odom_from_%d" % (r, sr), Odometry, queue_size=10)
        for i in range(self.n):
            rospy.Subscriber("/drone_%d/odom" % i, Odometry,
                             lambda m, i=i: self._on_odom(i, m))
        self.timer = rospy.Timer(rospy.Duration(0.05), self._tick)
        rospy.loginfo("comm_model: %d links, latency=%.0fms jitter=%.0fms "
                      "drop=%.3f burst=%.3f len=%s",
                      len(self.links), self.latency * 1000, self.jitter * 1000,
                      self.dropout, self.burst_prob, self.burst_len)

    def _on_odom(self, s, msg):
        now = rospy.get_time()
        for r in range(self.n):
            if r == s:
                continue
            st = self.links[(s, r)]
            rng = st["rng"]
            # 输入侧丢包：突发剩余优先，其次独立丢包率
            if st["burst"] > 0:
                st["burst"] -= 1
                continue
            if rng.random() < self.dropout:
                continue
            # 入队：release_time 单调递增（同链路 FIFO 不超车）；队列有界（延迟窗
            # ~2-3 条，40 为安全上限，超限丢最老）
            rel = max(st["prev"] + 1e-6,
                      now + self.latency + rng.uniform(0.0, self.jitter))
            st["prev"] = rel
            st["queue"].append((rel, msg))
            if len(st["queue"]) > 40:
                st["queue"].popleft()

    def _tick(self, event):
        now = rospy.get_time()
        for (s, r), st in self.links.items():
            rng = st["rng"]
            # 突发起始（每链路每秒 burst_prob；50ms tick → 乘 0.05）
            if st["burst"] == 0 and rng.random() < self.burst_prob * 0.05:
                st["burst"] = rng.randint(self.burst_len[0], self.burst_len[1])
            # 释放到期消息：发布最老 q[0]（= latency 前发出的那条），popleft
            q = st["queue"]
            if q and q[0][0] <= now:
                self.pubs[(s, r)].publish(q[0][1])
                q.popleft()

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        CommModelNode().run()
    except rospy.ROSInterruptException:
        pass
