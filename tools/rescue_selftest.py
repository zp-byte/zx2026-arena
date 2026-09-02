#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/rescue_selftest.py — 恢复链路仲裁修复对独立自测（不依赖 ROS 运行时）。

M1 rescue_mutex（goal-seal 触发互斥）+ M2 bounce_corridor（逃逸方向走廊化）：
  R1 M1 关态逐位：旗标关时恒不拦截（baseline 行为逐位保留）
  R2 M1 de 活跃拦截：逃逸进行中 goal-seal 禁发（drone5 病灶主项）
  R3 M1 plan_fail 拦截：规划连续失败（含 confirm 窗口）期间禁发
  R4 M1 通路保留：两状态皆清时放行（周期性试探不锁死）
  R5 M2 关态基线：旗标关时选 goal 对齐窄方向（原评分逐位）
  R6 M2 走廊偏好：旗标开时同可达改选宽走廊对角方向
  R7 M2 中性等价：点云空窗时裕度恒 1.0，开态选择与关态一致
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


# ---- M1 环境 -----------------------------------------------------------------
def make_rm(rm_enabled, de_active, plan_fail_since):
    s = types.SimpleNamespace()
    s._rm_enabled = rm_enabled
    s._de_active = de_active
    s._plan_fail_since = plan_fail_since
    s._gs_mutex_block = types.MethodType(H._gs_mutex_block, s)
    return s


# R1 关态逐位：两种危险状态组合都必须放行（与旧行为一致）
for da, pf in [(True, None), (False, 123.0), (True, 123.0)]:
    s = make_rm(False, da, pf)
    assert s._gs_mutex_block() is False, (da, pf)
print("R1 PASS")

# R2 de 活跃 → 拦截
s = make_rm(True, True, None)
assert s._gs_mutex_block() is True
print("R2 PASS")

# R3 plan_fail 连续 → 拦截（de confirm 窗口内同样挡住）
s = make_rm(True, False, 100.0)
assert s._gs_mutex_block() is True
print("R3 PASS")

# R4 双清 → 放行
s = make_rm(True, False, None)
assert s._gs_mutex_block() is False
print("R4 PASS")


# ---- M2 环境：空栅格 + 西侧走廊收窄点云 ---------------------------------------
# 16 均匀方向全部 reach=3.0（栅格无占用），区分度全在点云：
#   A(西, goal 对齐): (−2.0,±0.46) (−2.5,±0.46) —— 入口 1m 窗外收窄，
#     二值检查过、可达段最小侧距 0.46
#   B(西北 135°): 同点云侧距 >1.0 → 裕度满分
# goal_bias=1.5 下关态 A 胜（4.5 vs 4.06）；开态走廊项 B 反超（4.96 vs 5.06）
CLOUD = [(x, y, 2.5) for x in (-2.0, -2.5) for y in (-0.46, 0.46)]


def make_bc(bc_enabled, cloud):
    s = types.SimpleNamespace()
    s.res = 0.5
    s.g_x0, s.g_y0 = -5.0, -5.0
    s.g_nx, s.g_ny = 20, 20
    s._gocc_infl = np.zeros((20, 20), dtype=bool)
    s._g_to_cell = types.MethodType(H._g_to_cell, s)
    s.est_pos = np.array([0.0, 0.0, 2.5])
    s.odom = np.array([0.0, 0.0, 2.5])
    s.goal = np.array([-4.0, 0.0])
    s.cloud = cloud
    s.scene = types.SimpleNamespace(drone_radius=0.35)
    s._de_ray = 3.0
    s._de_ndirs = 16
    s._de_pierce = 8
    s._de_free_need = 3
    s._de_goal_bias = 1.5
    s._de_cloud_r = 1.0
    s._bc_enabled = bc_enabled
    s._bc_w = 1.0
    s._de_pick_dir = types.MethodType(H._de_pick_dir, s)
    s._bc_corridor_margin = types.MethodType(H._bc_corridor_margin, s)
    return s


def ang(d):
    return math.atan2(d[1], d[0])


# R5 关态基线：goal 对齐窄方向胜
s = make_bc(False, CLOUD)
d = s._de_pick_dir()
assert d is not None and abs(ang(d) - math.pi) < 0.01, ang(d)
print("R5 PASS")

# R6 开态走廊偏好：同可达改选宽走廊对角
s = make_bc(True, CLOUD)
d = s._de_pick_dir()
assert d is not None and abs(ang(d) - 3 * math.pi / 4) < 0.01, ang(d)
print("R6 PASS")

# R7 中性等价：点云空窗裕度恒 1.0，开态选择回到关态结果
s = make_bc(True, [])
d = s._de_pick_dir()
assert d is not None and abs(ang(d) - math.pi) < 0.01, ang(d)
s0 = make_bc(False, [])
d0 = s0._de_pick_dir()
assert abs(ang(d) - ang(d0)) < 1e-9
print("R7 PASS")

print("ALL PASS (R1-R7)")
