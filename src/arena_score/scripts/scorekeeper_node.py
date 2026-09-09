#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_score::scorekeeper_node — 比赛口径计分（全局 1 份）。

科目三：S = S1 + S2（满分 100）。
  * S1 投放分（封顶 60）：裁判侧在 /drone_<i>/payload/command 释放时刻
    快照真值 odom 判平台（scoring.judge_drop，与机载 MATCH 双重独立）——
    最近平台箱色≠箱色 → wrong_color；色同但释放点水平偏出平台投影
    (0.6m) → off_bucket；色同且投影内 → correct（+10/箱）。
  * S2 完成分（封顶 40）：时限内返回起降区降落的机数阶梯
    （/zx2026/task_update LANDED 事件计数；退赛机剔除——rule_monitor P5）。
旧 type 加权分/路径质量奖励/穿越区扣分移出总分，仅保留遥测字段。
发布 /zx2026/score/<drone_id> (Score)，赛毕写 /tmp/zx2026_score_*.yaml。
"""
import os
import time
import math

import rospy
from std_msgs.msg import Bool, String, UInt8, Int32
from nav_msgs.msg import Odometry

from zx2026_common import config as cfg
from zx2026_common import scoring
from zx2026_common.scene import Scene
from zx2026_common.msg import Mission, Score, TaskUpdate


class ScorekeeperNode:
    def __init__(self):
        rospy.init_node("scorekeeper_node", anonymous=True)
        rules = cfg.load("competition_rules.yaml")
        self.score_cfg = rules.get("score", {})
        self.scene_cfg = cfg.load("scene_topology.yaml")
        self.scene = Scene()
        self.n_drones = self.scene.drone_count

        # ---- 比赛口径计分参数 ----
        self.s1_per = int(self.score_cfg.get("s1_per_correct", 10))
        self.s1_cap = int(self.score_cfg.get("s1_cap", 60))
        ladder = self.score_cfg.get("s2_ladder", {}) or {}
        self.s2_ladder = ({int(k): int(v) for k, v in ladder.items()}
                          if ladder else {6: 40, 5: 30, 4: 25, 3: 15,
                                          2: 10, 1: 5, 0: 0})
        self.drop_tol = float(self.score_cfg.get("drop_tol_m", 0.6))

        # 路径质量与穿越区（遥测，不入总分）
        self.path_quality_cfg = self.scene_cfg.get("score", {}).get("path_quality", {})
        self.crossing_cfg = self.scene_cfg.get("score", {}).get("crossing_zone", {})
        self.pq_weight = float(self.path_quality_cfg.get("weight", 0.0))
        self.pq_enabled = bool(self.path_quality_cfg.get("straightness_bonus", False))
        self.crossing_required = bool(self.crossing_cfg.get("required", False))
        self.crossing_miss_penalty = int(self.crossing_cfg.get("miss_penalty", 0))

        self.mission = {}        # drone_id -> Mission
        self.match = {}          # 机载匹配结果（遥测）
        self.dropped = {}        # payload/done 机载投放事件（遥测）
        self.judged = {}         # 裁判判平台 verdict（S1 权威）
        self.drops = {}          # 判平台详情
        self.landed = {}         # S2 touchdown 事件
        self.retired = set()     # 退赛机（rule_monitor 接入）
        self.corridor_missed = set()  # 返程未穿林（剥 S2 计数，S1 保留）
        self.crossed_zone = {}
        self.first_correct = {}
        self.mission_done = {}
        self.odom_pose = {}      # 最新真值位置（释放时刻快照）

        self.path_odom = {}
        self.path_start = {}
        self.path_stats = {}
        self._reported = False

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
            rospy.Subscriber(ns + "/payload/command", UInt8,
                             lambda m, i=i: self._on_command(i, m))
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
            # 合规通道（P5 rule_monitor）
            rospy.Subscriber(ns + "/rule/retire", String,
                             lambda m, i=i: self.mark_retired(i, m.data))
            self.match[i] = "NONE"
            self.dropped[i] = False
            self.judged[i] = None
            self.drops[i] = None
            self.landed[i] = False
            self.crossed_zone[i] = False
            self.first_correct[i] = None
            self.mission_done[i] = False
            self.odom_pose[i] = None
            self.path_odom[i] = []
            self.path_stats[i] = {}
        rospy.Subscriber("/zx2026/task_update", TaskUpdate, self._on_task_update)
        rospy.Subscriber("/zx2026/rule/corridor_miss", Int32, self._on_corridor_miss)
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.loginfo("scorekeeper_node: %d drones (S1 cap=%d, S2 ladder=%s)",
                      self.n_drones, self.s1_cap,
                      dict(sorted(self.s2_ladder.items(), reverse=True)))

    # ---------------------------------------------------------------- callbacks
    def _on_mission(self, i, msg):
        self.mission[i] = msg

    def _set_match(self, i, val):
        self.match[i] = val

    def _set_crossed(self, i, val):
        self.crossed_zone[i] = val

    def _on_command(self, i, msg):
        """释放时刻（裁判口径）：快照真值位置判平台，独立于机载 MATCH。"""
        if self.judged[i] is not None:
            return
        m = self.mission.get(i)
        p = self.odom_pose.get(i)
        if m is None or p is None:
            rospy.logwarn("drone %d release command but no odom/mission, skip", i)
            return
        release_xy = (p[0], p[1])
        box_color = m.box_color or self.scene.color_for_type(m.payload_type)
        # 最近平台 = 物理上"投到的平台"（释放点正下方）
        best = min(self.scene.drop_points,
                   key=lambda dp: (dp.xyz[0] - release_xy[0]) ** 2
                   + (dp.xyz[1] - release_xy[1]) ** 2)
        verdict = scoring.judge_drop(release_xy, (best.xyz[0], best.xyz[1]),
                                     best.color, box_color, tol_m=self.drop_tol)
        off = math.hypot(release_xy[0] - best.xyz[0], release_xy[1] - best.xyz[1])
        self.judged[i] = verdict
        self.drops[i] = {"verdict": verdict, "offset": round(off, 3),
                         "bucket": best.id, "bucket_color": best.color,
                         "box_color": box_color,
                         "release_xy": [round(release_xy[0], 3), round(release_xy[1], 3)],
                         "t": round(rospy.get_time(), 2)}
        # 修复历史 bug：首次正确投放时间在判定时即记录（原在 DONE 才记/可能 None）
        if verdict == scoring.VERDICT_CORRECT and self.first_correct[i] is None:
            self.first_correct[i] = rospy.get_time()
        rospy.loginfo("drone %d release judged %s (platform %d %s, offset %.2f)",
                      i, verdict, best.id, best.color, off)
        self._publish_score(i)

    def _on_task_update(self, msg):
        if msg.mission_state != "LANDED":
            return
        i = msg.drone_id
        if not self.landed.get(i, False):
            self.landed[i] = True
            rospy.loginfo("drone %d LANDED counted for S2", i)
            self._publish_score(i)

    def _on_payload_done(self, i, msg):
        if msg.data and not self.dropped[i]:
            self.dropped[i] = True

    def _on_corridor_miss(self, msg):
        """返程未穿林（rule_monitor 裁决）：该机不计入 S2 阶梯，S1 保留。"""
        i = msg.data
        if i not in self.corridor_missed:
            self.corridor_missed.add(i)
            rospy.logwarn("drone %d corridor MISS — excluded from S2 count", i)
            self._publish_score(i)

    def _on_phase(self, i, phase):
        if phase == "EXECUTE" and not self.path_odom[i]:
            self.path_odom[i] = []
            self.path_stats[i] = {}
        elif phase == "DONE":
            self.mission_done[i] = True
            self._finalize_path(i)
            self._publish_score(i)
        elif phase == "FAILED":
            self.mission_done[i] = True
            self._finalize_path(i)
            self._publish_score(i)

    def _on_odom(self, i, msg):
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
        self.odom_pose[i] = pos
        if self._started and i in self.mission and not self.mission_done[i] \
                and i not in self.retired:
            self.path_odom[i].append(pos)

    def _on_state(self, msg):
        if msg.data == "P4_EXECUTE":
            self._started = True
        elif msg.data == "DONE" and not self._reported:
            # 时限封卷（run8 法证）：时限截断下未完成机永远到不了终态，
            # "全队 mission_done" 触发等不到——state DONE 即比赛定格口径，
            # 此刻封卷写报告（幂等，与全队终态触发共用 _reported 锁）
            self._reported = True
            self._write_report()

    # ---------------------------------------------------------------- retired (P5)
    def mark_retired(self, i, reason=""):
        """rule_monitor 退赛裁决入口：该机终局、S2 剔除、S1 保留。"""
        if i in self.retired:
            return
        self.retired.add(i)
        self.mission_done[i] = True
        self._finalize_path(i)
        rospy.logwarn("drone %d RETIRED (%s)", i, reason)
        self._publish_score(i)

    # ---------------------------------------------------------------- path helpers
    def _finalize_path(self, i):
        pts = self.path_odom[i]
        if len(pts) < 2:
            self.path_stats[i] = {"length": 0.0, "straight": 0.0, "straightness": 0.0}
            return
        length = 0.0
        for j in range(1, len(pts)):
            dx = pts[j][0] - pts[j - 1][0]
            dy = pts[j][1] - pts[j - 1][1]
            length += math.hypot(dx, dy)
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
    def _s1_of(self, i):
        """单机 S1：verdict==correct 计 10，否则 0。"""
        return self.s1_per if self.judged[i] == scoring.VERDICT_CORRECT else 0

    def _publish_score(self, i):
        verdict = self.judged[i]
        correct = 1 if verdict == scoring.VERDICT_CORRECT else 0
        wrong = 1 if verdict in (scoring.VERDICT_WRONG_COLOR,
                                 scoring.VERDICT_OFF_BUCKET) else 0
        s1_i = self._s1_of(i)
        s = Score()
        s.drone_id = i
        s.correct_drops = correct
        s.wrong_drops = wrong
        s.score = s1_i
        s.completion_time = rospy.get_time() if self.mission_done[i] else -1.0
        s.first_correct_time = self.first_correct[i] if self.first_correct[i] is not None else -1.0
        s.mission_state = ("RETIRED" if i in self.retired
                           else "DONE" if self.mission_done[i] else "RUNNING")
        s.s1 = s1_i
        s.retire_state = 1 if i in self.retired else 0
        self.pub[i].publish(s)

        pq = self.path_stats.get(i, {})
        summary = "drone=%d s1=%d verdict=%s landed=%s crossed=%s len=%.1f straightness=%.3f" % (
            i, s1_i, verdict or "none", str(self.landed[i]),
            str(self.crossed_zone[i]), pq.get("length", 0.0), pq.get("straightness", 0.0))
        self.pub_summary.publish(String(data=summary))

        if all(self.mission_done.values()) and self.mission_done:
            if not self._reported:
                self._reported = True
                self._write_report()

    def _write_report(self):
        try:
            import yaml
            s1_total = min(sum(self._s1_of(i) for i in range(self.n_drones)),
                           self.s1_cap)
            correct_n = sum(1 for i in range(self.n_drones)
                            if self.judged[i] == scoring.VERDICT_CORRECT)
            wrong_n = sum(1 for i in range(self.n_drones)
                          if self.judged[i] in (scoring.VERDICT_WRONG_COLOR,
                                                scoring.VERDICT_OFF_BUCKET))
            landed_n = sum(1 for v in self.landed.values() if v)
            # S2：退赛机/返程未穿林机剔除计数（S1 保留 = retire_score_keep 口径）
            eligible = [i for i in range(self.n_drones)
                        if i not in self.retired and i not in self.corridor_missed]
            s2 = scoring.s2_from_landed(
                sum(1 for i in eligible if self.landed.get(i)), self.s2_ladder)
            total = s1_total + s2

            rep = {
                "total": total,
                "s1_total": s1_total,
                "s2": s2,
                "landed_n": landed_n,
                "s2_eligible_n": len(eligible),
                "correct_n": correct_n,
                "wrong_n": wrong_n,
                "retired": sorted(self.retired),
                "corridor_missed": sorted(self.corridor_missed),
                "drops": {str(i): self.drops.get(i) for i in range(self.n_drones)},
                "scores": {},
            }
            for i in range(self.n_drones):
                pq = self.path_stats.get(i, {})
                rep["scores"][i] = {
                    "s1": self._s1_of(i),
                    "verdict": self.judged[i],
                    "match": self.match[i],
                    "landed": self.landed[i],
                    "retired": i in self.retired,
                    "crossed_zone": self.crossed_zone[i],
                    "path_length": pq.get("length", 0.0),
                    "straight_line_distance": pq.get("straight", 0.0),
                    "straightness": pq.get("straightness", 0.0),
                    "first_correct": self.first_correct[i],
                }
            path = "/tmp/zx2026_score_%d.yaml" % int(time.time())
            with open(path, "w") as f:
                yaml.safe_dump(rep, f, allow_unicode=True)
            # total= 前缀被 matrix_run/w1_flip_gate/gazebo_e2e 正则消费，必须保留
            self.pub_summary.publish(String(
                data="total=%d s1=%d s2=%d landed=%d correct=%d wrong=%d %s"
                     % (total, s1_total, s2, landed_n, correct_n, wrong_n, path)))
            rospy.loginfo("scorekeeper: report written to %s (total=%d s1=%d s2=%d "
                          "landed=%d)", path, total, s1_total, s2, landed_n)
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
