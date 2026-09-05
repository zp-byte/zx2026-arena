#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/cg_selftest.py — W3 塌缩卫兵 _collapse_guard 自测（不依赖 ROS 运行时）。

场景（run 214546/194437/live 返航冻结形态）：
  A 健康基指令（≥min_cmd）→ 原样返回（不干预）
  B 塌缩基指令 + 健康长路径 → 沿路径向前找 ≥min_target 航点接管，
    模值=floor_speed，cmd[2] 不动
  C 塌缩基指令 + 退化短路径（全在身边）→ pick=-1 直指 goal
  D 规划失败中 / 无路径 / 近 goal / 低净空 → 均不接管
"""
import importlib.util
import math
import types

import numpy as np

spec = importlib.util.spec_from_file_location(
    "nav_node", "/home/ubuntu/zx2026_arena_ws/src/arena_nav/scripts/nav_node.py")
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)
H = nav.NavNode
# 免 init_node：吞掉触发日志
nav.rospy.loginfo_throttle = lambda *a, **k: None

RES = 0.5
G_X0, G_Y0 = -10.0, -10.0
FLOOR = 0.8
MIN_TARGET = 0.75


def make_self(gpath, plan_fail=None, goal=(8.0, 3.0, 2.5),
              odom=(0.0, 0.0, 2.5), clearance=5.0):
    s = types.SimpleNamespace()
    s.drone_id = 0
    s._gocc_infl = None
    s.est_pos = np.array([0.0, 0.0, 2.5])   # est 格 = (20,20)
    s.odom = np.array(odom)
    s.goal = np.array(goal)
    s._gpath = gpath
    s._plan_fail_since = plan_fail
    s._cg_min_cmd = 0.3
    s._cg_floor_speed = FLOOR
    s._cg_min_target = MIN_TARGET
    s._cg_dgoal_min = 1.5
    s._cg_clear_min = 1.0
    s.res = RES
    s.g_x0, s.g_y0 = G_X0, G_Y0
    s.g_nx, s.g_ny = 40, 40
    s._g_to_cell = lambda x, y, _s=s: H._g_to_cell(_s, x, y)
    s._g_to_world = lambda ix, iy, _s=s: H._g_to_world(_s, ix, iy)
    return s


def w(ix, iy):
    return (G_X0 + (ix + 0.5) * RES, G_Y0 + (iy + 0.5) * RES)


def run(case, s, cmd):
    out = H._collapse_guard(s, np.array(cmd, dtype=float), 100.0,
                            s._cg_clear_min + 0.01 if case != "lowclr" else 0.5)
    return out


fails = []


def check(name, cond):
    print(("PASS " if cond else "FAIL ") + name)
    if not cond:
        fails.append(name)


# ---- A 健康基指令不干预 ------------------------------------------------------
path_long = [(20 + k, 20) for k in range(16)]          # 向 +x 前进的 16 格
s = make_self(path_long)
cmd0 = np.array([-1.49, 0.16, 0.0])
out = H._collapse_guard(s, cmd0.copy(), 100.0, 5.0)
check("A healthy cmd unchanged",
      np.allclose(out, cmd0, atol=1e-12))

# ---- B 塌缩 + 长路径 → 沿路径接管 ---------------------------------------------
s = make_self(path_long)
cmd_bad = np.array([0.0073, -0.0051, 0.3])
out = H._collapse_guard(s, cmd_bad.copy(), 100.0, 5.0)
tx, ty = w(21, 20)                                     # 第一格 ≥0.75m
un = math.hypot(tx, ty)
expect = np.array([tx / un * FLOOR, ty / un * FLOOR, 0.3])
check("B rescued along path (dir+mag)",
      np.allclose(out, expect, atol=1e-9))
check("B cmd_z untouched", abs(out[2] - 0.3) < 1e-12)

# ---- C 塌缩 + 退化短路径 → 直指 goal ------------------------------------------
s = make_self([(20, 20), (20, 20), (20, 20)])
out = H._collapse_guard(s, np.array([0.01, 0.0, 0.0]), 100.0, 5.0)
gx, gy = 8.0, 3.0
un = math.hypot(gx, gy)
expect = np.array([gx / un * FLOOR, gy / un * FLOOR, 0.0])
check("C degenerate path -> direct goal", np.allclose(out, expect, atol=1e-9))

# ---- D 各类不接管 --------------------------------------------------------------
s = make_self(path_long, plan_fail=0.0)
out = H._collapse_guard(s, np.array([0.01, 0.0, 0.0]), 100.0, 5.0)
check("D1 plan_fail -> unchanged", np.allclose(out, [0.01, 0.0, 0.0]))

s = make_self(None)
out = H._collapse_guard(s, np.array([0.01, 0.0, 0.0]), 100.0, 5.0)
check("D2 no path -> unchanged", np.allclose(out, [0.01, 0.0, 0.0]))

s = make_self(path_long, goal=(1.0, 0.5, 2.5))         # d_goal≈1.12 < 1.5
out = H._collapse_guard(s, np.array([0.01, 0.0, 0.0]), 100.0, 5.0)
check("D3 near goal -> unchanged", np.allclose(out, [0.01, 0.0, 0.0]))

s = make_self(path_long)
out = H._collapse_guard(s, np.array([0.01, 0.0, 0.0]), 100.0, 0.5)
check("D4 low clearance -> unchanged", np.allclose(out, [0.01, 0.0, 0.0]))

print("== cg_selftest %s ==" % ("ALL PASS" if not fails else
                                "FAILED: %s" % fails))
raise SystemExit(1 if fails else 0)
