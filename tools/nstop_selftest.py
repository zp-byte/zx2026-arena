#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/nstop_selftest.py — 近停区（near_stop_zone）独立自测（不依赖 ROS 运行时）。

验证 _nstop_scale 与 _apply_vcap 联动：
  N1 关态逐位原行为：scale 恒 1.0，vcap 与旧实现逐位一致
  N2 衰减曲线：zone 外/边界 1.0，线性降到 deadband 处 0，goal 内钳 0
  N3 开态限速：近 goal 贴脸 → 地板衰减，vcap 低于旧值且随距离单调
  N4 de 守卫：逃逸进行中不衰减（地板是安全项）
  N5 goal 缺失守卫：goal None → 1.0
"""
import importlib.util
import types

import numpy as np

spec = importlib.util.spec_from_file_location(
    "nav_node", "/home/ubuntu/zx2026_arena_ws/src/arena_nav/scripts/nav_node.py")
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)
H = nav.NavNode


def make_self(nstop_enabled, goal=(10.0, 0.0, 2.5), odom=(0.0, 0.0, 2.5),
              de_active=False):
    s = types.SimpleNamespace()
    s.odom = np.array(odom, dtype=float)
    s.goal = goal
    s._de_active = de_active
    s._dam_enabled = False
    s._nstop_enabled = nstop_enabled
    s._nstop_zone_r = 1.0
    s._nstop_deadband = 0.2
    s.closed_max_vel = 1.5
    s._nstop_scale = types.MethodType(H._nstop_scale, s)
    s._apply_vcap = types.MethodType(H._apply_vcap, s)
    return s


# ---- N1 关态逐位原行为 --------------------------------------------------------
for geo in ((10.0, 0.0, 2.5), (0.3, 0.0, 2.5), (0.15, 0.0, 2.5)):
    assert make_self(False, goal=geo)._nstop_scale() == 1.0
# vcap 旧口径：cmd 2.0 使封顶生效（vcap 只封顶不加速）
for clr, expect in ((3.0, 1.5),            # 远障不减速
                    (0.2, 1.5 * 0.35)):    # 贴脸 floor 0.35 兜底
    out = make_self(False)._apply_vcap(np.array([2.0, 0.0, 0.0]), clr)
    assert abs(np.linalg.norm(out) - expect) < 1e-9, (clr, np.linalg.norm(out))
print("N1 PASS")

# ---- N2 衰减曲线 ---------------------------------------------------------------
s = make_self(True)
for dg, expect in ((10.0, 1.0),    # zone 外
                   (1.0, 1.0),     # 边界（>= zone_r 不衰减）
                   (0.6, 0.5),     # 线性段中点 (0.6-0.2)/0.8
                   (0.2, 0.0),     # deadband 处归零
                   (0.1, 0.0)):    # goal 内钳 0
    s.odom = np.array([10.0 - dg, 0.0, 2.5])
    v = s._nstop_scale()
    assert abs(v - expect) < 1e-9, (dg, v)
print("N2 PASS")

# ---- N3 开态限速：地板衰减，vcap 严于旧值且随距离单调 ---------------------------
cmd = np.array([2.0, 0.0, 0.0])
outs = []
for dg in (0.9, 0.6, 0.3, 0.15):
    s = make_self(True, goal=(dg, 0.0, 2.5))
    outs.append(np.linalg.norm(s._apply_vcap(cmd.copy(), 0.2)))
# 贴脸 clr=0.2 → clr/2=0.1；旧值恒 1.5*0.35=0.525
for o in outs:
    assert o < 0.525, ("开态应严于旧值 0.525", outs)
assert outs[-1] < outs[0], ("应随接近 goal 单调收紧", outs)
# deadband 内（dg=0.15）地板归零：vcap 只剩 clearance/2 项
assert abs(outs[-1] - 1.5 * 0.1) < 1e-9, outs[-1]
print("N3 PASS")

# ---- N4 de 守卫：逃逸中不衰减 ---------------------------------------------------
s = make_self(True, goal=(0.15, 0.0, 2.5), de_active=True)
assert s._nstop_scale() == 1.0
out = s._apply_vcap(np.array([2.0, 0.0, 0.0]), 0.2)
assert abs(np.linalg.norm(out) - 1.5 * 0.35) < 1e-9
print("N4 PASS")

# ---- N5 goal 缺失守卫 -----------------------------------------------------------
s = make_self(True, goal=None)
assert s._nstop_scale() == 1.0
print("N5 PASS")

print("ALL SELFTEST PASS")
