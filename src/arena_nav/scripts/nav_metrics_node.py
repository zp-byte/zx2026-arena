#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""arena_nav::nav_metrics_node — P0 导航指标收集器（全局单实例，纯观测）。

订阅 /drone_<i>/nav_metrics（Float32MultiArray，布局见 nav_node）与 /zx2026/state，
落盘 run_logs/：
  nav_metrics_ts.csv           时间序列追加（run_seed, wall_t, sim_t, drone, ...）
  nav_metrics_latest.yaml      滚动快照（每 20s 覆盖，中途被杀也留有数据）
  nav_metrics_<启动ts>.yaml    DONE 定稿快照
DONE 时 loginfo 一行 NAV_METRICS_SUMMARY 供 run_verify 日志 grep。

不发布任何控制话题，对仿真行为零影响。
"""
import csv
import os
import time

import rospy
import yaml
from std_msgs.msg import String, Float32MultiArray

from zx2026_common import config as cfg
from zx2026_common.scene import Scene

# 与 nav_node __init__ 注释中的 11 项布局一一对应
FIELDS = ["collisions", "stuck_s", "stuck_n", "stuck_max_s", "flips",
          "dist_m", "min_clear_m", "max_speed", "sim_t", "speed", "clear"]


class NavMetrics(object):
    def __init__(self):
        rospy.init_node("nav_metrics", anonymous=True)
        self.drone_count = Scene().drone_count
        self.run_seed = int(cfg.load("sim_settings.yaml").get("run_seed", 0))
        self.state = "?"
        self.data = {i: None for i in range(self.drone_count)}
        self._final_written = False
        self._last_snap = 0.0

        logdir = os.path.expanduser("~/zx2026_arena_ws/run_logs")
        os.makedirs(logdir, exist_ok=True)
        self.ts = time.strftime("%Y%m%d_%H%M%S")
        self.yaml_path = os.path.join(logdir, "nav_metrics_%s.yaml" % self.ts)
        self.latest_path = os.path.join(logdir, "nav_metrics_latest.yaml")
        self.csv_path = os.path.join(logdir, "nav_metrics_ts.csv")
        new_csv = not os.path.exists(self.csv_path)
        self.csv_f = open(self.csv_path, "a", newline="")
        self.csv = csv.writer(self.csv_f)
        if new_csv:
            self.csv.writerow(["run_seed", "wall_t", "drone"] + FIELDS)

        for i in range(self.drone_count):
            rospy.Subscriber("/drone_%d/nav_metrics" % i, Float32MultiArray,
                             lambda m, i=i: self._on_metrics(i, m))
        rospy.Subscriber("/zx2026/state", String, self._on_state)
        rospy.loginfo("nav_metrics: %d drones seed=%d csv=%s",
                      self.drone_count, self.run_seed, self.csv_path)

    # ---- callbacks ------------------------------------------------------------
    def _on_metrics(self, i, msg):
        self.data[i] = list(msg.data)
        self.csv.writerow([self.run_seed, round(time.time(), 1), i]
                          + [round(float(v), 3) for v in msg.data])

    def _on_state(self, msg):
        self.state = msg.data

    # ---- 落盘 -----------------------------------------------------------------
    def _snapshot(self):
        snap = {"run_seed": self.run_seed, "state": self.state,
                "collected_wall": time.strftime("%Y-%m-%d %H:%M:%S"),
                "drones": {}}
        for i in range(self.drone_count):
            if self.data[i] is None:
                continue
            snap["drones"][str(i)] = dict(zip(
                FIELDS, [round(float(v), 3) for v in self.data[i]]))
        return snap

    def _write(self, path):
        with open(path, "w") as f:
            yaml.safe_dump(self._snapshot(), f, allow_unicode=True, sort_keys=False)

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if time.time() - self._last_snap > 20.0:
                self._last_snap = time.time()
                try:
                    self._write(self.latest_path)
                    self.csv_f.flush()
                except OSError as e:
                    rospy.logwarn("nav_metrics: snapshot write failed: %s", e)
            if self.state == "DONE" and not self._final_written:
                self._final_written = True
                try:
                    self._write(self.yaml_path)
                    self.csv_f.flush()
                    rospy.loginfo("NAV_METRICS_SUMMARY: %s", yaml.safe_dump(
                        self._snapshot()["drones"], default_flow_style=True))
                except OSError as e:
                    rospy.logwarn("nav_metrics: final write failed: %s", e)
            rate.sleep()
        self.csv_f.close()


if __name__ == "__main__":
    try:
        NavMetrics().run()
    except rospy.ROSInterruptException:
        pass
