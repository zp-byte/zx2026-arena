#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/veto_selftest.py — P2 指令否决层 _apply_veto 自测（不依赖 ROS 运行时）。

场景（v0=当前位置原点，走廊半宽 = 0.25+0.1 = 0.35m，margin=0.3，t_react=0.2，a=2.5）：
  V0 无点云           → 指令原样
  V1 点前方 1.0m      → room=0.7, v_allow=1.87 > 1.5 → 不触发
  V2 点前方 0.6m      → room=0.3, v_allow=1.22 < 1.5 → 缩到 0.816
  V3 点前方 0.35m+v_now=1.0 → room=0.35-0.3-0.2 <0 → 停（scale=0）
  V4 点真侧向 (0.4,0.05)：垂轨 0.4 > 0.35 走廊外 → 不触发（贴近绕行不受罚）
  V5 远离指令：点在前方 (0.5,0) 但 cmd 反向 → t=-0.5 ≤0 跳过 → 不触发
  V6 身后点 (-0.2,0)，cmd 前向 → t≤0 跳过 → 不触发（近旁归反应层管）
  V7 前方 0.8m + v_now=1.2 → room=0.26, v_allow=1.14 → 缩到 0.760
"""
import importlib.util
import math
import types

import numpy as np

spec = importlib.util.spec_from_file_location(
    "nav_node", "/home/ubuntu/zx2026_arena_ws/src/arena_nav/scripts/nav_node.py")
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)
nav.rospy.loginfo = lambda *a, **k: None  # shim：未 init_node 时静默
H = nav.NavNode


def make_self(cloud, cmd_v, v_now=0.0):
    s = types.SimpleNamespace()
    s._vt_enabled = True
    s._vt_t_react = 0.2
    s._vt_a_brake = 2.5
    s._vt_margin = 0.3
    s._vt_lat_margin = 0.1
    s._vt_ray = 2.5
    s._vt_scale_min = 0.35
    s._vt_vetoes = 0
    s.drone_id = 0
    s.odom = np.array([0.0, 0.0, 2.0])
    s.scene = types.SimpleNamespace(drone_radius=0.25)
    s.cloud = cloud
    s._v_est = np.array([v_now, 0.0, 0.0])
    return s


def run(s, cmd):
    return H._apply_veto(s, np.array(cmd, dtype=float))


def close(a, b, tol=1e-3):
    return abs(a - b) <= tol


# V0 空点云
r = run(make_self([], 1.5), [1.5, 0.0, 0.0])
assert close(np.linalg.norm(r[:2]), 1.5), "V0 fail: 空点云不应缩"
print("V0 pass: 空点云不缩")

# V1 前方 1.0m 不触发
r = run(make_self([(1.0, 0.0, 2.0)], 1.5), [1.5, 0.0, 0.0])
assert close(np.linalg.norm(r[:2]), 1.5), "V1 fail"
print("V1 pass: 前方 1.0m 包络内不触发 (v_allow=%.2f)" % math.sqrt(2 * 2.5 * 0.7))

# V2 前方 0.6m 缩到 v_allow=1.2247
r = run(make_self([(0.6, 0.0, 2.0)], 1.5), [1.5, 0.0, 0.0])
n = np.linalg.norm(r[:2])
assert close(n, math.sqrt(2 * 2.5 * 0.3), 1e-2), "V2 fail: got %.3f" % n
assert close(r[1], 0.0), "V2 fail: 方向不应改变"
print("V2 pass: 前方 0.6m 缩到 %.3f (期望 %.3f)" % (n, math.sqrt(1.5)))

# V3 前方 0.35m + v_now=1.0 → 缩到地板 0.35（不再全停：保留 vcap 同款爬行）
r = run(make_self([(0.35, 0.0, 2.0)], 1.5, v_now=1.0), [1.5, 0.0, 0.0])
assert close(np.linalg.norm(r[:2]), 1.5 * 0.35), "V3 fail"
print("V3 pass: 近点+v_now 大 → 地板爬行 %.3f (期望 0.525)"
      % np.linalg.norm(r[:2]))

# V4 真侧向点 (0.05,0.4)：t≈0、垂轨 0.4 > 走廊 0.35 → 贴近绕行不受罚
r = run(make_self([(0.05, 0.4, 2.0)], 1.5), [1.5, 0.0, 0.0])
assert close(np.linalg.norm(r[:2]), 1.5), "V4 fail"
print("V4 pass: 侧向 0.4m 走廊外不触发")

# V5 远离指令永不阻挡
r = run(make_self([(0.5, 0.0, 2.0)], 1.5), [-1.5, 0.0, 0.0])
assert close(np.linalg.norm(r[:2]), 1.5), "V5 fail"
print("V5 pass: 远离障碍不否决")

# V6 身后点不约束前向
r = run(make_self([(-0.2, 0.0, 2.0)], 1.5), [1.5, 0.0, 0.0])
assert close(np.linalg.norm(r[:2]), 1.5), "V6 fail"
print("V6 pass: 身后点(t≤0)不约束")

# V7 v_now 滞后项参与包络
r = run(make_self([(0.8, 0.0, 2.0)], 1.5, v_now=1.2), [1.5, 0.0, 0.0])
n = np.linalg.norm(r[:2])
want = math.sqrt(2 * 2.5 * max(0.0, 0.8 - 0.3 - 1.2 * 0.2))
assert close(n, want, 1e-2), "V7 fail: got %.3f want %.3f" % (n, want)
print("V7 pass: v_now=1.2 时包络缩到 %.3f (期望 %.3f)" % (n, want))

print("VETO SELFTEST PASS")
