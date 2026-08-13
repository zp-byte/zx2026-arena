#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_score::scorekeeper_node — 计分与报告（全局 1 份）。

订阅 6 机投放确认 + 匹配结果 + 任务信息，按 competition_rules.yaml 计分：
  * 正确投放（MATCH 且 payload/done）→ 加分（按 type 不同分值）
  * 路径质量奖励：路径越短、越直，额外加分
  * 未通过穿越区 → 扣分
  * 错误投放（TIMEOUT 但已投放）→ 罚分/0
发布 /zx2026/score/<drone_id> (Score)，赛事结束写报告到 /tmp/zx2026_score_*.yaml。
"""
import os
import time
import math

import rospy
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry

from zx2026_common import config as cfg
from zx2026_common.scene import Scene
from zx2026_common.msg import Mission, Score


class ScorekeeperNode:
    def __init__(self):
        rospy.init_node("scorekeeper_node", anonymous=True)
        rules = cfg.load("competition_rules.yaml")
        self.score_cfg = rules.get("score", {})
        self.scene_cfg = cfg.load("scene_topology.yaml")
        self.scene = Scene()
        self.n_drones = self.scene.drone_count

        # 路径质量与穿越区评分配置
        self.path_quality_cfg = self.scene_cfg.get("score", {}).get("path_quality", {})
        self.crossing_cfg = self.scene_cfg.get("score", {}).get("crossing_zone", {})
        self.pq_weight = float(self.path_quality_cfg.get("weight", 0.0))
        self.pq_enabled = bool(self.path_quality_cfg.get("straightness_bonus", False))
        self.crossing_required = bool(self.crossing_cfg.get("required", False))
        self.crossing_miss_penalty = int(self.crossing_cfg.get("miss_penalty", 0))

        self.mission = {}       # drone_id -> Mission
        self.match = {}         # drone_id -> "MATCH"/"TIMEOUT"/"NONE"
        self.dropped = {}       # drone_id -> True/False
        self.crossed_zone = {}  # drone_id -> True/False
        self.score = {}         # drone_id -> int
        self.first_correct = {}  # drone_id -> sim time
        self.mission_done = {}   # drone_id -> Bool
        self._started = False

        # 路径追踪
        self.path_odom = {}     # drone_id -> [(x,y,z,t), ...]
        self.path_start = {}    # drone_id -> (x,y,z,t)
        self.path_end = {}      # drone_id -> (x,y,z,t)
        self.path_stats = {}    # drone_id -> dict

        self.pub = {}
        self.pub_summary = rospy.Publisher("/zx2026/score_summary", String, queue_size=1, latch=True)
        self.pub_path_quality = {}
        for i in range(self.n_drones):
            ns = "/drone_%d" % i
            self.pub[i] = rospy.Publisher("/zx2026/score/%d" % i, Score, queue_size=1, latch=True)
            self.pub_path_quality[i] = rospy.Publisher("/zx2026/score/%d/path_quality" % i,
                                                        String, queue_size=1, latch=True)
            rospy.Subscriber(ns + "/payload/done", Bool,
                             lambda m, i=i: self._on_payload_done(i, m))
            rospy.Subscriber(ns + "/match/result", String,
                             lambda m, i=i: self._set_match(i, m.data))
            rospy.Subscriber(ns + "/mission/phase", String,
                             lambda m, i=i: self._on_phase(i, m.data))
            rospy.Subscriber("/zx2026/mission/%d" % i, Mission,
                             lambda m, i=i: self._on_mission(i, m))
            rospy.Subscriber(ns + "/odom", Odometry,
                             lambda m, i=i: self._on_odom(i, m))
            rospy.Subscriber(ns + "/mission/crossed_zone", Bool,
                             lambda m, i=i: self._set_crossed(i, m.data))
            self.score[i] = 0
            self.match[i] = "NONE"
            self.dropped[i] = False
            self.crossed_zone[i] = False
            self.first_correct[i] = None
            self.mission_done[i] = False
            self.path_odom[i] = []
            self.path_stats[i] = {}

        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.loginfo("scorekeeper_node: %d drones", self.n_drones)

    # ---------------------------------------------------------------- callbacks
    def _on_mission(self, i, msg):
        self.mission[i] = msg

    def _set_match(self, i, val):
        self.match[i] = val

    def _set_crossed(self, i, val):
        self.crossed_zone[i] = val

    def _on_payload_done(self, i, msg):
        if msg.data and not self.dropped[i]:
            self.dropped[i] = True
            self._finalize_path(i)
            self._score_drop(i)

    def _on_phase(self, i, phase):
        if phase == "EXECUTE" and not self.path_odom[i]:
            self.path_odom[i] = []
            self.path_start[i] = None
            self.path_stats[i] = {}
        elif phase == "DONE":
            self.mission_done[i] = True
            if self.first_correct[i] is None and self.dropped[i] and self.match[i] == "MATCH":
                self.first_correct[i] = rospy.get_time()
            self._publish_score(i)
        elif phase == "FAILED":
            self.mission_done[i] = True
            self._finalize_path(i)
            self._publish_score(i)

    def _on_odom(self, i, msg):
        if not self._started:
            return
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        # 仅在 EXECUTE 阶段记录路径
        phase_topic = rospy.get_param("~phase_topic", None)
        # 通过缓存的 mission phase 判断：若最近收到的是 EXECUTE 则记录
        # 这里简化为：只要有任务且未投放，就记录
        if i in self.mission and not self.dropped[i] and not self.mission_done[i]:
            if not self.path_odom[i]:
                self.path_start[i] = pos
            self.path_odom[i].append(pos)

    def _on_state(self, msg):
        if msg.data == "P4_EXECUTE":
            self._started = True

    # ---------------------------------------------------------------- path helpers
    def _finalize_path(self, i):
        """在投放或失败时计算路径统计。"""
        pts = self.path_odom[i]
        if len(pts) < 2:
            self.path_stats[i] = {"length": 0.0, "straight": 0.0, "straightness": 0.0}
            return
        # 实际飞行路径长度（水平距离累加）
        length = 0.0
        for j in range(1, len(pts)):
            dx = pts[j][0] - pts[j - 1][0]
            dy = pts[j][1] - pts[j - 1][1]
            length += math.hypot(dx, dy)
        # 起点到终点的直线距离
        start = pts[0]
        end = pts[-1]
        straight = math.hypot(end[0] - start[0], end[1] - start[1])
        straightness = straight / length if length > 1e-6 else 0.0
        self.path_stats[i] = {
            "length": length,
            "straight": straight,
            "straightness": straightness,
        }
        self._publish_path_quality(i)

    def _publish_path_quality(self, i):
        s = self.path_stats.get(i, {})
        txt = "drone=%d length=%.2f straight=%.2f straightness=%.3f crossed=%s" % (
            i, s.get("length", 0.0), s.get("straight", 0.0),
            s.get("straightness", 0.0), str(self.crossed_zone[i]))
        self.pub_path_quality[i].publish(String(data=txt))

    # ---------------------------------------------------------------- scoring
    def _score_drop(self, i):
        if i not in self.mission:
            return
        m = self.mission[i]
        base = int(self.score_cfg.get(m.payload_type, 0))
        if self.match[i] == "MATCH":
            self.score[i] += base
            rospy.loginfo("drone %d CORRECT drop (+%d)", i, base)
            # 路径质量奖励
            if self.pq_enabled:
                s = self.path_stats.get(i, {})
                st = s.get("straightness", 0.0)
                bonus = int(base * self.pq_weight * st)
                self.score[i] += bonus
                rospy.loginfo("drone %d path quality bonus +%d (straightness %.3f)",
                              i, bonus, st)
            # 穿越区惩罚
            if self.crossing_required and not self.crossed_zone[i]:
                self.score[i] += self.crossing_miss_penalty
                rospy.logwarn("drone %d missed crossing zone (%d)", i, self.crossing_miss_penalty)
            if self.first_correct[i] is None:
                self.first_correct[i] = rospy.get_time()
        else:
            penalty = int(self.score_cfg.get("wrong_drop_penalty", 0))
            self.score[i] += penalty
            rospy.logwarn("drone %d wrong/unknown drop (penalty %d)", i, penalty)
            # 穿越区惩罚也适用于错误投放
            if self.crossing_required and not self.crossed_zone[i]:
                self.score[i] += self.crossing_miss_penalty
        self._publish_score(i)

    def _publish_score(self, i):
        s = Score()
        s.drone_id = i
        s.correct_drops = 1 if (self.dropped[i] and self.match[i] == "MATCH") else 0
        s.wrong_drops = 1 if (self.dropped[i] and self.match[i] != "MATCH") else 0
        s.score = self.score[i]
        s.completion_time = rospy.get_time() if self.mission_done[i] else -1.0
        s.first_correct_time = self.first_correct[i] if self.first_correct[i] is not None else -1.0
        s.mission_state = "DONE" if self.mission_done[i] else "RUNNING"
        self.pub[i].publish(s)

        pq = self.path_stats.get(i, {})
        summary = "drone=%d score=%d correct=%d wrong=%d crossed=%s len=%.1f straightness=%.3f" % (
            i, s.score, s.correct_drops, s.wrong_drops,
            str(self.crossed_zone[i]), pq.get("length", 0.0), pq.get("straightness", 0.0))
        self.pub_summary.publish(String(data=summary))

        if all(self.mission_done.values()) and self.mission_done:
            self._write_report()

    def _write_report(self):
        try:
            import yaml
            rep = {"scores": {}, "total": 0}
            for i in range(self.n_drones):
                pq = self.path_stats.get(i, {})
                rep["scores"][i] = {
                    "score": self.score[i],
                    "match": self.match[i],
                    "crossed_zone": self.crossed_zone[i],
                    "path_length": pq.get("length", 0.0),
                    "straight_line_distance": pq.get("straight", 0.0),
                    "straightness": pq.get("straightness", 0.0),
                    "first_correct": self.first_correct[i],
                }
                rep["total"] += self.score[i]
            path = "/tmp/zx2026_score_%d.yaml" % int(time.time())
            with open(path, "w") as f:
                yaml.safe_dump(rep, f, allow_unicode=True)
            self.pub_summary.publish(String(data="total=%d %s" % (rep["total"], path)))
            rospy.loginfo("scorekeeper: report written to %s (total=%d)",
                          path, rep["total"])
        except Exception as e:
            rospy.logwarn("scorekeeper report failed: %s", e)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = ScorekeeperNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
