#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/plan_selftest.py — _plan_global 目标容忍孔径规划自测（不依赖 ROS 运行时）。

场景：投放点清空被膨胀环封死（goal 本格也幻影占用）：
  A 环上有真实缺口（树间间隙）→ 孔径规划应给出直达 goal 的路径
  B 环封闭（孔径与外部自由区不连通）→ 退化为最近可达格 alt
  C goal 自由 → 普通规划直达 goal
"""
import importlib.util
import types

import numpy as np

spec = importlib.util.spec_from_file_location(
    "nav_node", "/home/ubuntu/zx2026_arena_ws/src/arena_nav/scripts/nav_node.py")
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)
H = nav.NavNode

RES = 0.5
G = (19, 20)  # goal 格；世界 = (-7.5 + (i+0.5)*0.5, ...) = (2.25, 2.75)


def make_self(occ, est, goal_world):
    s = types.SimpleNamespace()
    ny, nx = occ.shape
    s._gocc_infl = occ
    s.est_pos = np.array(est)
    s.goal = goal_world
    s.res = RES
    s.g_x0 = -nx * RES / 2.0
    s.g_y0 = -ny * RES / 2.0
    s.g_nx = nx
    s.g_ny = ny
    s._g_to_cell = lambda x, y, _s=s: H._g_to_cell(_s, x, y)
    s._grid_astar = lambda occ, a, b: H._grid_astar(occ, a, b)
    s._nearest_free_grid = lambda occ, i, j: H._nearest_free_grid(occ, i, j)
    return s


def sealed_grid(gap=False):
    """30x30 格。goal 及其 4 格内全占用（幻影内芯 + 膨胀环）。
    gap=True 时在 +x 方向 Chebyshev 3-4 处开 2 格真实缺口。"""
    occ = np.zeros((30, 30), dtype=bool)
    gx, gy = G
    for iy in range(30):
        for ix in range(30):
            if max(abs(ix - gx), abs(iy - gy)) <= 4:
                occ[iy, ix] = True
    if gap:
        occ[gy, gx + 3] = False
        occ[gy, gx + 4] = False
    return occ


goal_w = (-7.5 + (G[0] + 0.5) * RES, -7.5 + (G[1] + 0.5) * RES)

# A: 有缺口 → 孔径路径应达 goal 本格
pA = H._plan_global(make_self(sealed_grid(gap=True), (-6.0, -6.0), goal_w))
print("A path:", "None" if pA is None else "len=%d end=%s" % (len(pA), pA[-1]))
assert pA is not None and tuple(pA[-1]) == G, "A fail: 孔径路径应达 goal"

# B: 环封闭 → 退化 alt（最近可达格），路径终点在环外
pB = H._plan_global(make_self(sealed_grid(gap=False), (-6.0, -6.0), goal_w))
print("B path:", "None" if pB is None else "len=%d end=%s" % (len(pB), pB[-1]))
assert pB is not None and tuple(pB[-1]) != G, "B fail: 封闭环应退化 alt"

# C: goal 自由 → 普通路径达 goal
pC = H._plan_global(make_self(np.zeros((30, 30), dtype=bool), (-6.0, -6.0), goal_w))
print("C path:", "None" if pC is None else "len=%d end=%s" % (len(pC), pC[-1]))
assert pC is not None and tuple(pC[-1]) == G, "C fail"

print("PLAN SELFTEST PASS")
