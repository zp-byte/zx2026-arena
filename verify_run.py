#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""端到端验证：启动全栈 → 等 fleet_ready → /zx2026/start → 观测 150s 全程。

记录: /zx2026/state 状态时间线、每 5s 的 drone_0/1 位置、mission/phase、
      match/result、score_summary，输出到 stdout + /tmp/zx2026_verify.txt。
用法: bash -c 'source devel/setup.bash && python3 verify_run.py'
"""
import math
import time

import rospy
import rosparam
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class Verifier:
    def __init__(self):
        self.state = "?"
        self.phases = {}
        self.match = {}
        self.odom = {}
        self.score_summary = None
        self.fleet_ready = False

    def _on_state(self, m):
        self.state = m.data

    def _on_phase(self, i):
        def cb(m, i=i):
            self.phases[i] = m.data
        return cb

    def _on_match(self, i):
        def cb(m, i=i):
            self.match[i] = m.data
        return cb

    def _on_odom(self, i):
        def cb(m, i=i):
            p = m.pose.pose.position
            self.odom[i] = (round(p.x, 2), round(p.y, 2), round(p.z, 2))
        return cb

    def _on_score(self, m):
        self.score_summary = m.data

    def _on_fleet(self, m):
        self.fleet_ready = m.data

    def run(self):
        rospy.init_node("verify_run", anonymous=True)
        for i in range(6):
            rospy.Subscriber("/drone_%d/odom" % i, Odometry, self._on_odom(i))
            rospy.Subscriber("/drone_%d/mission/phase" % i, String, self._on_phase(i))
            rospy.Subscriber("/drone_%d/match/result" % i, String, self._on_match(i))
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.Subscriber("/zx2026/score_summary", String, self._on_score)
        rospy.Subscriber("/zx2026/fleet_ready", Bool, self._on_fleet)

        out = open("/tmp/zx2026_verify.txt", "w")
        log = lambda *a: (print(*a), out.write(" ".join(str(x) for x in a) + "\n"), out.flush())

        # 等 fleet_ready（最多 30s）
        t0 = time.time()
        while not self.fleet_ready and time.time() - t0 < 30:
            rospy.sleep(0.2)
        log("fleet_ready =", self.fleet_ready)

        # 等 P2_WAIT，然后 start
        t0 = time.time()
        while self.state != "P2_WAIT" and time.time() - t0 < 20:
            rospy.sleep(0.2)
        try:
            srv = rospy.ServiceProxy("/zx2026/start", Trigger)
            r = srv()
            log("start service:", r.success, r.message)
        except Exception as e:
            log("start service FAIL:", e)

        # P6：观测窗跟随时限参数化（时限+60s，上限 1600；DONE 早退保留）
        try:
            from zx2026_common import config as _cfg
            tl = float(_cfg.load("competition_rules.yaml").get("time_limit_s", 600.0))
        except Exception:
            tl = 600.0
        observe_s = min(tl + 60.0, 1600.0)
        log("=== observation (%.0fs, sample every 5s) ===" % observe_s)
        last_state = None
        t0 = time.time()
        while time.time() - t0 < observe_s:
            if self.state != last_state:
                last_state = self.state
                log("-- state ->", self.state, "@ %.0fs" % (time.time() - t0))
            o0 = self.odom.get(0)
            o1 = self.odom.get(1)
            if o0 or o1:
                log("  t=%.0f drone0=%s drone1=%s phase0=%s match0=%s"
                    % (time.time() - t0, o0, o1,
                       self.phases.get(0), self.match.get(0)))
            if self.state == "DONE":
                rospy.sleep(2.0)   # 封卷报告落盘窗口（scorekeeper 收 DONE 写 yaml）
                break
            rospy.sleep(5)

        log("=== final ===")
        log("state:", self.state)
        log("phases:", {k: v for k, v in sorted(self.phases.items())})
        log("match:", {k: v for k, v in sorted(self.match.items())})
        log("odom drone0:", self.odom.get(0))
        log("score_summary:", self.score_summary)
        # 通过判定（比赛口径 2026-09-09）：时限封卷语义下未完成机不再等终态，
        # "六机全 DONE+MATCH" 仅作 FULL 档对照打印。PASS = state DONE 时封卷
        # 成功：封卷报告落盘（mtime≥观测起点防旧 run 残留）+ 零退赛 + ≥1 机落地。
        full = (all(self.phases.get(i) == "DONE" for i in range(6))
                and all(self.match.get(i) == "MATCH" for i in range(6)))
        rep = None
        try:
            import glob
            import os as _os
            import yaml as _yaml
            cands = [f for f in glob.glob("/tmp/zx2026_score_*.yaml")
                     if _os.path.getmtime(f) >= t0 - 5.0]
            if cands:
                p = max(cands, key=_os.path.getmtime)
                rep = _yaml.safe_load(open(p))
        except Exception:
            rep = None
        log("full_completion:", full)
        log("report retired:", (rep or {}).get("retired"),
            "landed_n:", (rep or {}).get("landed_n"))
        ok = (self.state == "DONE" and rep is not None
              and not (rep.get("retired") or [])
              and (rep.get("landed_n") or 0) >= 1)
        log("VERDICT:", "PASS" if ok else "FAIL")
        out.close()


if __name__ == "__main__":
    try:
        Verifier().run()
    except rospy.ROSInterruptException:
        pass
