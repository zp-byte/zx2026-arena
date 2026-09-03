#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/avoid_selftest.py — 全史碰撞聚类驱动的避障优化对独立自测（无 ROS 运行时）。

M3 sep_obs_guard（分离增量障碍投影）+ M4 hotspot_inflate（热点树定向膨胀）：
  S1 M3 关态逐位：旗标关时输出与无守卫基线逐位一致
  S2 M3 指树投影：分离增量指向贴身树 → 指树分量被投影掉（残余 ⊥ 树向）
  S3 M3 远树中性：树超出 guard 范围 → 开态=关态逐位
  S4 M3 背树中性：分离增量背离贴身树 → 开态=关态逐位（不挡远离/切向）
  S5 M3 空云中性：点云空 → 开态=关态逐位
  H1 M4 关态基线：旗标关时热点格外圈不被占用
  H2 M4 定向膨胀：旗标开时热点格外圈被占用（+extra_cells）
  H3 M4 非热点隔离：非热点障碍的膨胀范围不因开态改变
  H4 M4 足迹保留：自机在热点上时足迹清除仍生效（起点不被占死锁）
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

DR = 0.35


def make_sep(soa_enabled, neighbor_xy, cloud):
    s = types.SimpleNamespace()
    s._soa_enabled = soa_enabled
    s.scene = types.SimpleNamespace(drone_radius=DR)
    s.odom = np.array([0.0, 0.0, 2.5])
    s.max_vel = 2.0
    s.sep_radius = 1.5
    s.sep_gain = 3.5
    s.neighbors = {1: (neighbor_xy[0], neighbor_xy[1], 2.5)}
    s.cloud = cloud
    s._sep_obs_guard = types.MethodType(H._sep_obs_guard, s)
    s._apply_separation = types.MethodType(H._apply_separation, s)
    return s


def sep_result(s):
    return s._apply_separation(np.array([1.0, 0.0, 0.0]))


# S1 关态逐位：有树同场景，开=守卫数学期望、关=原行为
TREE_S = [(0.0, -0.5, 2.5)]   # 正南 0.5m 贴身树
s_off = make_sep(False, (0.0, 1.0), TREE_S)     # 北邻 → 推向南（指树）
r_off = sep_result(s_off)
assert abs(r_off[1] - (0.0 - 1.0) / 1.0 * 3.5 * 0.5 / 1.5) < 1e-9, r_off
print("S1 PASS")

# S2 指树投影：南树 + 北邻（推向南）→ 南向分量清零，东向保持
s_on = make_sep(True, (0.0, 1.0), TREE_S)
r_on = sep_result(s_on)
assert abs(r_on[0] - 1.0) < 1e-9 and abs(r_on[1]) < 1e-9, r_on
print("S2 PASS")

# S3 远树中性：树在 guard 范围外 → 开=关
far = [(0.0, -2.0, 2.5)]
s_off = make_sep(False, (0.0, 1.0), far)
s_on = make_sep(True, (0.0, 1.0), far)
assert np.allclose(sep_result(s_on), sep_result(s_off), atol=0, rtol=0)
print("S3 PASS")

# S4 背树中性：树在增量反侧 → 开=关（不挡远离分量）
s_off = make_sep(False, (0.0, 1.0), [(0.0, 0.5, 2.5)])   # 树北，推离北邻=向南=背离树
s_on = make_sep(True, (0.0, 1.0), [(0.0, 0.5, 2.5)])
assert np.allclose(sep_result(s_on), sep_result(s_off), atol=0, rtol=0)
print("S4 PASS")

# S5 空云中性
s_off = make_sep(False, (0.0, 1.0), [])
s_on = make_sep(True, (0.0, 1.0), [])
assert np.allclose(sep_result(s_on), sep_result(s_off), atol=0, rtol=0)
print("S5 PASS")


# ---- M4 环境 ------------------------------------------------------------------
def make_hi(hi_enabled):
    s = types.SimpleNamespace()
    s._pc_enabled = False
    s._occ_cells = {(50, 50), (60, 60)}          # (50,50)=热点树，(60,60)=普通障碍
    s.inflation = 0.4
    s.res = 0.5
    s.g_x0, s.g_y0 = -25.0, -25.0
    s.g_nx, s.g_ny = 100, 100
    s._collision_obs = []
    s._gocc_infl = None
    s.est_pos = np.array([-20.0, -20.0, 2.5])    # 远离测试格
    s._hi_enabled = hi_enabled
    s._hi_extra = 2                              # 基础膨胀已占 ±1；extra=2 才触及 [52,50]
    s._hi_trees = [(0.0, 0.0)]                   # (0,0) → cell (50,50)
    s._dilate_rect = H._dilate_rect   # staticmethod：直接挂函数，勿 MethodType（会注入 self）
    s._g_to_cell = types.MethodType(H._g_to_cell, s)
    s._publish_occ_grid = types.MethodType(lambda self: None, s)
    s._rebuild_global_grid = types.MethodType(H._rebuild_global_grid, s)
    return s


# H1 关态基线：热点格外圈一格不被占用
s = make_hi(False)
s._rebuild_global_grid()
assert not s._gocc_infl[52, 50], "flag-off must not dilate hotspot"
assert s._gocc_infl[51, 50], "base dilation missing"
print("H1 PASS")

# H2 定向膨胀：开态热点格外圈一格被占用
s = make_hi(True)
s._rebuild_global_grid()
assert s._gocc_infl[52, 50], "hotspot +1 dilate missing"
assert s._gocc_infl[51, 50], "hotspot core missing"
print("H2 PASS")

# H3 非热点隔离：普通障碍膨胀范围不因开态改变
s0 = make_hi(False)
s0._rebuild_global_grid()
s1 = make_hi(True)
s1._rebuild_global_grid()
assert bool(s0._gocc_infl[62, 60]) == bool(s1._gocc_infl[62, 60])
assert not s1._gocc_infl[62, 60], "non-hotspot must not gain cells"
print("H3 PASS")

# H4 足迹保留：自机站在热点上，足迹清除仍生效（含外圈）
s = make_hi(True)
s.est_pos = np.array([0.0, 0.0, 2.5])
s._rebuild_global_grid()
assert not s._gocc_infl[50, 50] and not s._gocc_infl[51, 50], "footprint must clear"
print("H4 PASS")

print("ALL PASS (S1-S5, H1-H4)")
