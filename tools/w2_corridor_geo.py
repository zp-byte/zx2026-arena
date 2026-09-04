#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w2_corridor_geo.py — Wave2 立案：走廊带树列 + NE 投放角障碍查询。"""
import math
import os
import sys

WS = "/home/ubuntu/zx2026_arena_ws"
sys.path.insert(0, WS + "/src/zx2026_common/scripts")
from zx2026_common.scene import Scene  # noqa: E402

sc = Scene()

print("== 北缘走廊带树列 (y in [5.5, 9.5], 按 x 排序) ==")
trees = [ob for ob in sc.obstacles if ob.kind == "tree"
         and 5.5 <= ob.cy <= 9.5]
for ob in sorted(trees, key=lambda o: o.cx):
    print("  tree (%6.2f,%6.2f) trunk_r=%.2f" % (ob.cx, ob.cy, ob.trunk_r))
print("  count=%d" % len(trees))

print("== NE 投放角 (x>15, y>10) 障碍 ==")
for ob in sc.obstacles:
    cx = getattr(ob, "cx", None)
    cy = getattr(ob, "cy", None)
    if cx is None:
        lo = getattr(ob, "lo", None)
        hi = getattr(ob, "hi", None)
        if lo is None:
            continue
        cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    if cx > 15 and cy > 10:
        print("  %s (%6.2f,%6.2f) %s" % (ob.kind, cx, cy,
              "trunk_r=%.2f" % ob.trunk_r if ob.kind == "tree" else ""))

print("== 投放点 vs 停机坪 ==")
for dp in sorted(sc.drop_points, key=lambda d: d.xyz[1]):
    print("  drop %s xyz=%s" % (dp.id, dp.xyz))

print("== 碰撞点最近障碍 (matrix_w1b 三簇代表点) ==")
for label, (x, y) in (("北缘狭缝A", (-12.7, 6.4)), ("带内北缘", (5.2, 7.1)),
                      ("NE投放角", (19.7, 12.25)), ("带内东侧", (8.2, 2.8)),
                      ("西坪进场", (-19.7, 8.65))):
    best, bd = None, 1e9
    for ob in sc.obstacles:
        if ob.kind == "tree":
            d = math.hypot(x - ob.cx, y - ob.cy) - ob.trunk_r
        else:
            lo, hi = getattr(ob, "lo", None), getattr(ob, "hi", None)
            if lo is None:
                continue
            dx = max(lo[0] - x, 0.0, x - hi[0])
            dy = max(lo[1] - y, 0.0, y - hi[1])
            d = math.hypot(dx, dy)
        if d < bd:
            bd, best = d, ob
    cx = getattr(best, "cx", None)
    if cx is None:
        lo, hi = best.lo, best.hi
        cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    else:
        cy = best.cy
    print("  %-8s (%6.1f,%6.1f) -> %s (%6.2f,%6.2f) 表面距 %.2f"
          % (label, x, y, best.kind, cx, cy, bd))
