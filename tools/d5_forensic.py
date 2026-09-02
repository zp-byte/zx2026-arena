#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""d5_forensic.py — live run (2026-09-02) drone5 慢绕异常离线法证。

CSV 无 x/y，只有 dist_m/speed/clear/flips——用"里程率(实际推进速度) vs
车速 + 事件(碰撞/de/stall)时间线"定位慢绕形态：原地蹭(里程率~0)还是
绕远(里程率高但路径长)。
"""
import csv
import io
import time

PATH = "/home/ubuntu/zx2026_arena_ws/run_logs/nav_metrics_ts.csv"

raw = open(PATH, "rb").read().decode("utf-8", "replace").replace("\x00", "")
rows = []
for r in csv.DictReader(io.StringIO(raw, newline="")):
    try:
        rows.append((int(r["drone"]), float(r["wall_t"]), float(r["dist_m"]),
                     float(r["speed"]), float(r["clear"]), float(r["flips"]),
                     float(r["collisions"]), int(float(r["run_seed"])),
                     float(r["stuck_s"])))
    except Exception:
        pass
rows.sort(key=lambda t: t[1])

segs, cur = [], [rows[0]]
for prev, r in zip(rows, rows[1:]):
    if r[1] - prev[1] > 20:
        segs.append(cur)
        cur = []
    cur.append(r)
segs.append(cur)

print("=== 最近 5 个 run 段 ===")
for s in segs[-5:]:
    t0, t1 = s[0][1], s[-1][1]
    print("SEG %s -> %s (%.0fs) rows=%d drones=%s seed=%s"
          % (time.strftime("%m-%d %H:%M:%S", time.localtime(t0)),
             time.strftime("%H:%M:%S", time.localtime(t1)), t1 - t0, len(s),
             sorted(set(x[0] for x in s)), sorted(set(x[7] for x in s))))

# live 段 = 倒数第 2 段（最后一段是正在跑的 dam2 cell）
live = segs[-2]
t0 = live[0][1]
print("\n=== live 段 drone5 10s 桶剖面（对照 drone0）===")
print("t_off  d5_mgap  d5_spd  d5_clr  d5_flips  d5_col | d0_mgap d0_spd d0_clr")
b = {}
for r in live:
    if r[0] in (0, 5):
        b.setdefault(r[0], []).append(r)
d5, d0 = b[5], b[0]
step = 10.0
n = int((live[-1][1] - t0) / step) + 1


def buckets(arr):
    out = []
    for k in range(n):
        lo, hi = t0 + k * step, t0 + (k + 1) * step
        w = [x for x in arr if lo <= x[1] < hi]
        out.append(w)
    return out


b5, b0 = buckets(d5), buckets(d0)
for k in range(n):
    w5, w0 = b5[k], b0[k]
    if not w5:
        continue
    mg5 = w5[-1][2] - w5[0][2]
    sp5 = sum(x[3] for x in w5) / len(w5)
    cl5 = sum(x[4] for x in w5) / len(w5)
    fl5 = w5[-1][5] - w5[0][5]
    co5 = w5[-1][6] - w5[0][6]
    st5 = w5[-1][8] - w5[0][8]
    if w0:
        mg0 = w0[-1][2] - w0[0][2]
        sp0 = sum(x[3] for x in w0) / len(w0)
        cl0 = sum(x[4] for x in w0) / len(w0)
        d0s = "%.2f     %.2f   %.2f" % (mg0, sp0, cl0)
    else:
        d0s = "-"
    mark = ""
    if co5 > 0:
        mark += " COL+%g" % co5
    if st5 > 2:
        mark += " STUCK+%.0f" % st5
    if fl5 >= 3:
        mark += " FLIP+%g" % fl5
    print("%4.0fs  %6.2fm  %5.2f  %5.2f  %5g  %3g | %s%s"
          % (k * step, mg5, sp5, cl5, fl5, co5, d0s, mark))

# 汇总统计
tot = d5[-1]
print("\ndrone5: dist=%.1fm stuck=%.1fs flips=%g col=%g (t=%.0fs)"
      % (tot[2], tot[8], tot[5], tot[6], tot[1] - t0))
lo = [k * step for k, w in enumerate(b5) if w and (w[-1][2] - w[0][2]) < 0.3]
print("里程率<0.3m/s 的 10s 桶: %s" % lo)
