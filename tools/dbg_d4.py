#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""调试：seed-42 drone-4 各 run 的 CSV 轨迹分组（矩阵 B_tc42 卡死排查用）。"""
import csv

PATH = "/home/ubuntu/zx2026_arena_ws/run_logs/nav_metrics_ts.csv"
rows = []
_raw = open(PATH, "rb").read().replace(b"\x00", b"").decode("utf-8", "replace")
for r in csv.reader(_raw.splitlines()):
    if len(r) < 14:
        continue
    try:
        float(r[0])
    except ValueError:
        continue
    if r[0] == "42" and r[2] == "4":
        rows.append(r)

print("seed42 drone4 rows:", len(rows))
if not rows:
    raise SystemExit

print("wall range:", rows[0][1], "->", rows[-1][1])
runs = []
prev = None
for r in rows:
    t = float(r[1])
    if prev is None or t - prev > 60:
        runs.append([])
    runs[-1].append(r)
    prev = t
print("runs detected:", len(runs), [len(x) for x in runs])
last = runs[-1]
print("--- last run (every 25th row) ---")
for r in last[::25]:
    print("t=%s col=%s stuck=%.1f n=%s flips=%s dist=%.1f spd=%.2f clr=%s"
          % (r[1][-7:], r[3], float(r[4]), r[5], r[7], float(r[8]),
             float(r[12]), r[13]))
print("LAST:", last[-1][1:])
