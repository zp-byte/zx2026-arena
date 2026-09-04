#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w2_ne_check.py — Wave2 立案：北 fence / NE 角近邻查询。

回答: RETURN NE_drop 三起 (19.6~19.9,12.2~12.3)/(14.0,12.3)/(-14.4,11.8)
撞的是 NE 树还是北 fence；西坪 4 起的 fence 归属复核。
"""
import math
import sys

WS = "/home/ubuntu/zx2026_arena_ws"
sys.path.insert(0, WS + "/src/zx2026_common/scripts")
from zx2026_common.scene import Scene  # noqa: E402

sc = Scene()

print("== 非树障碍一览 ==")
for ob in sc.obstacles:
    if ob.kind == "tree":
        continue
    lo, hi = getattr(ob, "lo", None), getattr(ob, "hi", None)
    if lo is None:
        print("  %s (no lo/hi) %r" % (ob.kind, ob))
    else:
        print("  %-8s lo=%s hi=%s" % (ob.kind, lo, hi))


def dist_to(ob, x, y):
    if ob.kind == "tree":
        return math.hypot(x - ob.cx, y - ob.cy) - ob.trunk_r
    lo, hi = getattr(ob, "lo", None), getattr(ob, "hi", None)
    if lo is None:
        return 1e9
    dx = max(lo[0] - x, 0.0, x - hi[0])
    dy = max(lo[1] - y, 0.0, y - hi[1])
    return math.hypot(dx, dy)


print("== 代表点最近障碍 ==")
pts = [("R-NE a", 19.9, 12.3), ("R-NE b", 19.8, 12.2), ("R-NE c", 14.0, 12.3),
       ("R-NW ", -14.4, 11.8), ("R-slit", 5.5, 7.6),
       ("W-pad a", -19.1, 8.7), ("W-pad b", -20.4, 8.6),
       ("D6 起步", 19.5, 10.0)]
for label, x, y in pts:
    best, bd = None, 1e9
    for ob in sc.obstacles:
        d = dist_to(ob, x, y)
        if d < bd:
            bd, best = d, ob
    lo = getattr(best, "lo", None)
    if best.kind == "tree":
        where = "tree(%6.2f,%6.2f) r=%.2f" % (best.cx, best.cy, best.trunk_r)
    else:
        where = "%s lo=%s hi=%s" % (best.kind, lo, getattr(best, "hi", None))
    print("  %-8s (%6.1f,%6.1f) -> %-34s 距 %.2f" % (label, x, y, where, bd))

print("== drop6 返航起步方向检查: drop6 到 pad 的直线 vs 北缘树列 ==")
drop6 = (19.5, 10.0)
pads = sorted(((ob.cx, ob.cy) for ob in sc.obstacles if ob.kind == "tree"), key=lambda t: -t[0])  # noqa: E501
# pad 不是树——用 drop 点/已知 pad 坐标 -22.5/-26 带内; 这里只列树列对直线的穿越
# 直线 drop6(-19.5? pad 取 (-22.5, 0.0) 代表) 简化: y 从 10 线性降到 0
print("  直线 (19.5,10)->(-22.5,0) 在北缘树列 x 段 y 值:")
for tx in (-12.2, -7.9, 0.1, 5.4, 9.2, 13.0):
    t = (tx - 19.5) / (-22.5 - 19.5)
    y_line = 10.0 + t * (0.0 - 10.0)
    print("    x=%6.1f -> y_line=%5.2f (树列 y 5.7-7.0)" % (tx, y_line))
