#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/de_selftest.py — _de_pick_dir / _de_tick 独立自测（不依赖 ROS 运行时，只 import）。

用合成占用栅格验证死端逃逸方向评分的 7 个场景：
  S0 空地：应选朝 goal 方向
  S1 碰撞标记壳包住自机：应穿壳逃出（pierce 预算内）
  S2 壳 + goal 侧堵墙：应绕开被堵方向
  S3 四面全堵（ray 内不可行）：应返回 None（悬停重试）
  S4 靠场地边界：不应逃向边界外
  S5 (P2) goal 对角沿线上有点云真树：该方向被走廊检查淘汰
  S6 (P2) 点云在走廊外：对角方向不受罚
  S7 (P2) 四周沿线上全是真树点云：全淘汰 → None 悬停
状态机（P5 退出迟滞，显式时刻驱动 _de_tick）：
  H1 正常迟滞退出：规划恢复后仍保持逃逸 ≥exit_hyst_s 才交还
  H2 迟滞窗口内规划再失败：计时清零重计
  H3 规划快速翻转（成功窗口 <confirm_s）：应根本不触发
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


def make_self(occ, est, goal, res=0.5, cloud=None):
    s = types.SimpleNamespace()
    ny, nx = occ.shape
    s._gocc_infl = occ
    s.est_pos = np.array(est)
    s.goal = goal
    s.res = res
    s.g_x0 = -nx * res / 2.0
    s.g_y0 = -ny * res / 2.0
    s.g_nx = nx
    s.g_ny = ny
    s._de_ndirs = 16
    s._de_pierce = 8
    s._de_free_need = 3
    s._de_goal_bias = 1.5
    s._de_ray = 3.0
    s._de_cloud_r = 1.0
    # M2 bounce_corridor 桩补（落地时未同步 selftest，存量破损修复）
    s._bc_enabled = False
    s._bc_w = 1.0
    # W1 rescue_quiet 桩补（de 退出爬坡窗钩子读旗；关=不置窗）
    s._swq_enabled = False
    s._swq_ramp = 2.5
    s._swq_until = -1e9
    s.scene = types.SimpleNamespace(drone_radius=0.25)
    s.odom = np.array([est[0], est[1], 2.5])
    s.cloud = cloud if cloud is not None else []
    s._g_to_cell = lambda x, y, _s=s: H._g_to_cell(_s, x, y)
    return s


def ang_of(d):
    return math.atan2(d[1], d[0])


# S0 空地
occ0 = np.zeros((20, 20), dtype=bool)
d0 = H._de_pick_dir(make_self(occ0, (0.0, 0.0), (8.0, 8.0)))
print("S0 dir:", d0, "ang=%.3f" % ang_of(d0))
assert d0 is not None and abs(ang_of(d0) - math.pi / 4) < 0.5, "S0 fail"

# S1 5x5 碰撞标记壳（世界系 [-1.0,1.5]^2），自机在中心，goal 对角
occ1 = np.zeros((20, 20), dtype=bool)
occ1[8:13, 8:13] = True
d1 = H._de_pick_dir(make_self(occ1, (0.0, 0.0), (8.0, 8.0)))
print("S1 dir:", d1, "ang=%.3f" % ang_of(d1))
assert d1 is not None and abs(ang_of(d1) - math.pi / 4) < 0.5, "S1 fail"

# S2 壳 + +x 侧整排堵死（模拟树墙），goal 在 +x
occ2 = np.zeros((20, 20), dtype=bool)
occ2[8:13, 8:13] = True
occ2[8:13, 13:20] = True
d2 = H._de_pick_dir(make_self(occ2, (0.0, 0.0), (9.0, 0.0)))
print("S2 dir:", d2, "ang=%.3f" % ang_of(d2))
assert d2 is not None and not (-0.5 < ang_of(d2) < 0.5), "S2 fail"

# S3 10x10 大堵死区（ray 内不可行）
occ3 = np.zeros((20, 20), dtype=bool)
occ3[5:15, 5:15] = True
d3 = H._de_pick_dir(make_self(occ3, (0.0, 0.0), (9.0, 9.0)))
print("S3 dir:", d3)
assert d3 is None, "S3 fail"

# S4 自机靠 +x 场地边界（hx=5.0），goal 在远处 -x
occ4 = np.zeros((20, 20), dtype=bool)
occ4[8:13, 8:13] = True
d4 = H._de_pick_dir(make_self(occ4, (4.4, 0.0), (-9.0, 0.0)))
print("S4 dir:", d4, "ang=%.3f" % ang_of(d4))
assert d4 is not None and abs(ang_of(d4)) > 1.0, "S4 fail"

# S5 (P2 修复) 壳场景 + goal 对角沿线上 0.7m 有点云真树：
# 对角方向（±22.5° 邻域同被走廊扫到）被点云淘汰，应改选其他可行方向
occ5 = np.zeros((20, 20), dtype=bool)
occ5[8:13, 8:13] = True
d5 = H._de_pick_dir(make_self(
    occ5, (0.0, 0.0), (8.0, 8.0),
    cloud=[(0.5, 0.5, 2.5)]))
print("S5 dir:", d5, "ang=%.3f" % ang_of(d5))
assert d5 is not None and abs(ang_of(d5) - math.pi / 4) > 0.3, "S5 fail"

# S6 (P2 修复) 同壳但点云点在走廊外（垂轨 0.53 > 0.35）：
# 对角方向不受罚，仍应选 goal 对角
d6 = H._de_pick_dir(make_self(
    occ5, (0.0, 0.0), (8.0, 8.0),
    cloud=[(0.05, 0.8, 2.5)]))
print("S6 dir:", d6, "ang=%.3f" % ang_of(d6))
assert d6 is not None and abs(ang_of(d6) - math.pi / 4) < 0.3, "S6 fail"

# S7 (P2 修复) 四周沿线上 1.0m 全是真树点云：全方向被点云淘汰 → None 悬停
# （12 点/30° 间隔：任意 16 向方向上最大垂轨 1.0·sin(15°)=0.26 < 走廊 0.35）
d7 = H._de_pick_dir(make_self(
    occ5, (0.0, 0.0), (8.0, 8.0),
    cloud=[(math.cos(a), math.sin(a), 2.5) for a in
           [2 * math.pi * k / 12 for k in range(12)]]))
print("S7 dir:", d7)
assert d7 is None, "S7 fail"


# ---- P5 退出迟滞状态机（_de_tick，显式时刻驱动；rospy 日志打桩） ------------
_logs = []
nav.rospy.loginfo = lambda fmt, *a: _logs.append(str(fmt) % a if a else str(fmt))
nav.rospy.loginfo_throttle = lambda p, fmt, *a: _logs.append(str(fmt) % a if a else str(fmt))


def make_tick_self():
    """_de_tick 所需最小状态（数值与 sim_settings 默认一致）。"""
    s = types.SimpleNamespace()
    s.drone_id = 0
    s._de_active = False
    s._de_dir = None
    s._de_t0 = -1e9
    s._de_total_t0 = -1e9
    s._de_fail_t0 = None
    s._de_ok_t0 = None
    s._de_confirm = 0.5
    s._de_exit_hyst = 2.5
    s._de_dwell = 1.2
    s._plan_fail_since = None
    # W1 rescue_quiet 桩补（de 退出爬坡窗钩子读旗；关=不置窗）
    s._swq_enabled = False
    s._swq_ramp = 2.5
    s._swq_until = -1e9
    s._de_pick_dir = lambda: (1.0, 0.0)
    s._de_dir_str = types.MethodType(H._de_dir_str, s)
    return s


def H1_normal_exit():
    """规划恢复后保持逃逸满 2.5s 才交还；窗口内不提前退出。"""
    s = make_tick_self()
    s._plan_fail_since = 100.0
    H._de_tick(s, 100.0)                      # 失败计时起点
    assert not s._de_active, "H1: 提前触发"
    H._de_tick(s, 100.6)                      # 0.6s >= confirm → 触发
    assert s._de_active, "H1: 应触发"
    s._plan_fail_since = None                 # 规划恢复（单次成功窗口）
    H._de_tick(s, 101.0)
    assert s._de_active, "H1: 单次成功不应立即退出"
    H._de_tick(s, 103.4)                      # 2.4s < 2.5s
    assert s._de_active, "H1: 未满迟滞不应退出"
    H._de_tick(s, 103.6)                      # 2.6s >= 2.5s
    assert not s._de_active, "H1: 满迟滞应退出"
    assert any("escape done" in m for m in _logs), "H1: 缺 done 日志"
    assert s._de_ok_t0 is None, "H1: 退出后应清计时器"


def H2_fail_reset():
    """迟滞窗口内规划再失败 → 计时清零重计，退出时刻顺延。"""
    s = make_tick_self()
    s._plan_fail_since = 200.0
    H._de_tick(s, 200.0)
    H._de_tick(s, 200.6)
    assert s._de_active, "H2: 应触发"
    s._plan_fail_since = None
    H._de_tick(s, 201.0)                      # ok_t0=201.0
    s._plan_fail_since = 202.0                # 窗口内再失败
    H._de_tick(s, 202.0)
    assert s._de_active, "H2: 再失败应保持逃逸"
    assert s._de_ok_t0 is None, "H2: 再失败应清成功计时"
    s._plan_fail_since = None
    H._de_tick(s, 204.0)                      # 重计起点 204.0
    H._de_tick(s, 206.4)                      # 2.4s < 2.5s → 未满
    assert s._de_active, "H2: 重计未满不应退出"
    H._de_tick(s, 206.6)                      # 2.6s >= 2.5s
    assert not s._de_active, "H2: 重计满应退出"


def H3_rapid_flip_never_engages():
    """规划成败快速翻转（成功窗口 <confirm_s）：de 根本不触发。"""
    s = make_tick_self()
    s._plan_fail_since = 300.0
    H._de_tick(s, 300.0)
    s._plan_fail_since = None
    H._de_tick(s, 300.3)                      # 失败 0.3s < confirm
    assert not s._de_active, "H3: 短失败不应触发"
    s._plan_fail_since = 300.4
    H._de_tick(s, 300.4)
    s._plan_fail_since = None
    H._de_tick(s, 300.7)                      # 又只失败 0.3s
    assert not s._de_active, "H3: 快速翻转不应累积触发"
    assert s._de_fail_t0 is None, "H3: 成功期应清失败计时"


H1_normal_exit()
print("H1 PASS")
H2_fail_reset()
print("H2 PASS")
H3_rapid_flip_never_engages()
print("H3 PASS")

print("ALL SELFTEST PASS")
