#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/dam_selftest.py — 漂移感知裕度（dam）独立自测（不依赖 ROS 运行时）。

验证 _dam_clearance 与 _apply_vcap：
  D1 无漂移恒等：eff == 真值 clearance，ero == 0
  D2 方向投影：只有"背向障碍"分量侵蚀裕度（朝向/垂直分量不侵蚀）
  D3 开态限速：全额侵蚀时 0.35 地板衰减到 0，vcap 用 eff（严于关态）
  D4 关态逐位原行为：vcap 公式与旧实现一致（含 floor 0.35 与不减速区）
  D5 部分侵蚀：地板按侵蚀占比连续衰减，eff/2 为主、地板兜底
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


def make_self(dam_enabled, drift=(0.0, 0.0, 0.0), cloud=None):
    s = types.SimpleNamespace()
    s.odom = np.array([0.0, 0.0, 2.5])
    s.cloud = cloud if cloud is not None else [(0.8, 0.0, 2.5)]  # 正右方 0.8m
    s.drift = np.array(drift)
    s._dam_enabled = dam_enabled
    s.closed_max_vel = 1.5
    s.drift_max = 0.3
    s._dam_clearance = types.MethodType(H._dam_clearance, s)
    s._apply_vcap = types.MethodType(H._apply_vcap, s)
    return s


# ---- D1 无漂移恒等 -----------------------------------------------------------
s = make_self(dam_enabled=True)
eff, ero = s._dam_clearance()
assert ero == 0.0 and abs(eff - 0.8) < 1e-9, (eff, ero)
print("D1 PASS")

# ---- D2 方向投影：仅背向分量侵蚀 ---------------------------------------------
s = make_self(True, drift=(-0.2, 0.0, 0.0))          # 背向障碍（障碍在 +x）
eff, ero = s._dam_clearance()
assert abs(ero - 0.2) < 1e-9 and abs(eff - 0.6) < 1e-9, (eff, ero)
s = make_self(True, drift=(0.2, 0.0, 0.0))           # 朝向障碍：est 更近，不侵蚀
eff, ero = s._dam_clearance()
assert ero == 0.0 and abs(eff - 0.8) < 1e-9, (eff, ero)
s = make_self(True, drift=(0.0, 0.2, 0.0))           # 垂直：投影 0
eff, ero = s._dam_clearance()
assert abs(ero) < 1e-12 and abs(eff - 0.8) < 1e-9, (eff, ero)
print("D2 PASS")

# ---- D3 开态：全额侵蚀 → 地板衰减到 0，vcap 用 eff（严于关态） -----------------
s = make_self(True, drift=(-0.3, 0.0, 0.0))          # ero=0.3=drift_max → floor=0
cmd = np.array([1.0, 0.0, 0.0])
out = s._apply_vcap(cmd.copy(), 0.8)
eff_vcap = 1.5 * (0.5 / 2.0)                          # eff=0.8-0.3=0.5
assert abs(np.linalg.norm(out) - eff_vcap) < 1e-9, np.linalg.norm(out)
s_off = make_self(False, drift=(-0.3, 0.0, 0.0))      # 同几何关态：floor 0.35 生效
out_off = s_off._apply_vcap(cmd.copy(), 0.8)          # clearance 0.8 → 1.5*0.4=0.6
assert abs(np.linalg.norm(out_off) - 0.6) < 1e-9
assert np.linalg.norm(out) < np.linalg.norm(out_off), "开态应更严"
print("D3 PASS")

# ---- D4 关态逐位原行为 --------------------------------------------------------
# cmd norm 2.0 使封顶真正生效（vcap 只封顶不加速）
for clr, expect in ((3.0, 1.5),     # 远障不减速
                    (1.0, 1.5 * 0.5),   # eff/2 主导（floor 只在 clr<0.7 bind）
                    (0.2, 1.5 * 0.35),  # 贴脸 floor 兜底
                    (1.5, 1.5 * 0.75)):  # eff/2 主导
    s = make_self(False)
    out = s._apply_vcap(np.array([2.0, 0.0, 0.0]), clr)
    assert abs(np.linalg.norm(out) - expect) < 1e-9, (clr, np.linalg.norm(out))
print("D4 PASS")

# ---- D5 部分侵蚀：地板连续衰减 ------------------------------------------------
s = make_self(True, drift=(-0.15, 0.0, 0.0))         # ero=0.15 → floor=0.175
out = s._apply_vcap(np.array([1.0, 0.0, 0.0]), 0.8)
# eff=0.65 → eff/2=0.325 > floor 0.175 → vcap=0.4875
assert abs(np.linalg.norm(out) - 1.5 * 0.325) < 1e-9
s = make_self(True, drift=(-0.15, 0.0, 0.0), cloud=[(0.25, 0.0, 2.5)])  # 贴脸
out = s._apply_vcap(np.array([1.0, 0.0, 0.0]), 0.25)
# eff=0.10 → eff/2=0.05 < floor 0.175 → 地板兜底 vcap=0.2625（旧值 0.525 的一半）
assert abs(np.linalg.norm(out) - 1.5 * 0.175) < 1e-9
print("D5 PASS")

print("ALL SELFTEST PASS")
