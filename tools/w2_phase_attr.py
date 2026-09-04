#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w2_phase_attr.py — Wave 2 立案证据：drone5 走廊碰撞相位归因。

输入: matrix_w1* 目录树（每 cell evidence.txt 首行 run_dir=<uuid>）
方法: 每机从 executor_N 日志建任务腿时间线（takeoff done → heading to via
      slot → crossed zone → return start → FAILED），nav_node_N 碰撞行按
      epoch 落进腿窗口 → 逐碰撞 (腿, 腿内 t, x, y)。
日志双格式并存: 主 .log=[rosout][INFO] <wall>，-stdout.log=[INFO] [<epoch>]。
wall 用本机时区换算回 epoch（同机自洽）；同事件跨文件按 (drone,x,y,秒) 去重。
用法: python3 tools/w2_phase_attr.py [matrix_dir ...]  # 缺省=run_logs/matrix_w1*
"""
import glob
import os
import re
import sys
import time
from collections import Counter

WS = "/home/ubuntu/zx2026_arena_ws"
ROS_LOG = os.path.expanduser("~/.ros/log")
RUN_LOGS = os.path.join(WS, "run_logs")

VERB = (r"takeoff done|heading to (?:via slot|crossing zone)"
        r"|crossed zone, heading to drop|FAILED|return start")
COL_EPOCH = re.compile(r"\[(\d+)\.[\d]+\]:?.*?nav_node: drone (\d) collision "
                       r"marker at \(([-\d.]+),([-\d.]+)\)")
COL_WALL = re.compile(r"\[rosout\]\[INFO\] ([\d\-]+ [\d:,]+): nav_node: drone "
                      r"(\d) collision marker at \(([-\d.]+),([-\d.]+)\)")
LEG_EPOCH = re.compile(r"\[(\d+)\.[\d]+\]:?.*?drone (\d) (" + VERB + r")")
LEG_WALL = re.compile(r"\[rosout\]\[INFO\] ([\d\-]+ [\d:,]+):.*?drone (\d) ("
                      + VERB + r")")
_WALL_CACHE = {}


def wall_to_epoch(s):
    if s not in _WALL_CACHE:
        try:
            _WALL_CACHE[s] = time.mktime(time.strptime(s, "%Y-%m-%d %H:%M:%S,%f"))
        except ValueError:
            _WALL_CACHE[s] = None
    return _WALL_CACHE[s]


def read(fp):
    try:
        with open(fp, encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


def run_dir_of(evidence_path):
    first = read(evidence_path).splitlines()
    if first and first[0].startswith("run_dir="):
        p = os.path.join(ROS_LOG, first[0].split("=", 1)[1])
        return p if os.path.isdir(p) else None
    return None


def _phase(verb):
    if verb.startswith("heading to"):
        return "APPROACH"
    if verb.startswith("crossed zone"):
        return "DROP_SIDE"
    if verb == "takeoff done":
        return "TAKEOFF_DONE"
    if verb == "return start":
        return "RETURN"
    return "FAILED"


def attr_run(run_dir):
    """返回 [(drone, epoch, phase, leg_t, x, y)]，跨文件去重。"""
    legs = {}  # drone -> [(epoch, phase)] 时间线
    for fp in glob.glob(os.path.join(run_dir, "executor_*-*.log")):
        for ln in read(fp).splitlines():
            m = LEG_EPOCH.search(ln) or LEG_WALL.search(ln)
            if not m:
                continue
            if m.re is LEG_WALL:
                ep = wall_to_epoch(m.group(1))
                if ep is None:
                    continue
                d, verb = m.group(2), m.group(3)
            else:
                ep, d, verb = float(m.group(1)), m.group(2), m.group(3)
            legs.setdefault(int(d), []).append((ep, _phase(verb)))
    for d in legs:
        legs[d].sort()

    seen = set()
    out = []
    for fp in glob.glob(os.path.join(run_dir, "nav_node_*-*.log")):
        for ln in read(fp).splitlines():
            m = COL_EPOCH.search(ln) or COL_WALL.search(ln)
            if not m:
                continue
            if m.re is COL_WALL:
                ep = wall_to_epoch(m.group(1))
                if ep is None:
                    continue
                d, x, y = m.group(2), m.group(3), m.group(4)
            else:
                ep, d, x, y = float(m.group(1)), m.group(2), m.group(3), m.group(4)
            key = (d, round(float(x), 1), round(float(y), 1), int(ep))
            if key in seen:
                continue
            seen.add(key)
            d, ep = int(d), float(ep)
            ph, lt = "PRE-TAKEOFF", -1.0
            for le, lp in legs.get(d, []):
                if le <= ep:
                    ph, lt = lp, ep - le
            out.append((d, ep, ph, lt, float(x), float(y)))
    out.sort(key=lambda r: (r[1], r[0]))
    return out


def band(x, y):
    """走廊地带归属（case 文档口径）。"""
    if x > 15 and y > 8:
        return "NE_drop"
    if 4.5 <= y <= 9.5 and -14 <= x <= 14:
        return "north_slit"
    if x < -15:
        return "west_pad"
    return "mid"


def main():
    dirs = sys.argv[1:] or sorted(glob.glob(os.path.join(RUN_LOGS, "matrix_w1*")),
                                  key=os.path.getmtime)
    rows_all = []
    print("%-30s %-2s %-12s %6s  %-14s %s" %
          ("cell", "d", "phase", "leg_t", "(x,y)", "epoch"))
    for md in dirs:
        mb = os.path.basename(md)
        for ev in sorted(glob.glob(os.path.join(md, "*", "evidence.txt"))):
            cell = mb + "/" + os.path.basename(os.path.dirname(ev))
            rd = run_dir_of(ev)
            if rd is None:
                print("%-30s  (run_dir missing)" % cell)
                continue
            for row in attr_run(rd):
                rows_all.append((cell,) + row)
                _, d, ep, ph, lt, x, y = rows_all[-1]
                print("%-30s %-2d %-12s %6.1f  (%6.1f,%6.1f)  %.0f"
                      % (cell, d, ph, lt, x, y, ep))
    print("\n== 相位 x 地带 汇总 (%d 碰撞) ==" % len(rows_all))
    c = Counter((ph, band(x, y)) for _, _, _, ph, _, x, y in rows_all)
    for (ph, b), n in sorted(c.items(), key=lambda kv: -kv[1]):
        print("  %-12s %-10s %d" % (ph, b, n))


if __name__ == "__main__":
    main()
