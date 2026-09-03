#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p4_selftest.py — P4 滑窗点缓存独立自测（不依赖 ROS 运行时，只 import）。

验证 _rebuild_global_grid 的滑窗选格逻辑：
  C0 关态逐位原行为：持久集合全量入图，_occ_seen 不参与
  C1 开态窗内保留 + 窗外衰减：新格在图，旧格被剪枝且不在图
  C2 碰撞标记持久：标记格（事件真值）不随窗衰减
  C3 自机足迹清除 + 膨胀：est 3x3 可通行、占用格邻域被膨胀
"""
import importlib.util
import types

import numpy as np

spec = importlib.util.spec_from_file_location(
    "nav_node", "/home/ubuntu/zx2026_arena_ws/src/arena_nav/scripts/nav_node.py")
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)
H = nav.NavNode

_NOW = [1000.0]
nav.rospy.get_time = lambda: _NOW[0]


def make_self(pc_enabled, ttl=45.0):
    s = types.SimpleNamespace()
    s.g_nx = 20
    s.g_ny = 20
    s.res = 0.5
    s.g_x0 = -5.0
    s.g_y0 = -5.0
    s.inflation = 0.4
    s._pc_enabled = pc_enabled
    s._pc_ttl = ttl
    # M4 hotspot_inflate 桩补（落地时未同步 selftest，存量破损修复）
    s._hi_enabled = False
    s._hi_extra = 1
    s._hi_trees = []
    s._occ_cells = set()
    s._occ_seen = {}
    s._collision_obs = set()
    s.est_pos = np.array([0.0, 0.0, 2.5])   # 中心 (10,10) 格
    s._dilate_rect = H._dilate_rect
    s._g_to_cell = types.MethodType(H._g_to_cell, s)
    s._publish_occ_grid = lambda: None
    return s


# ---- C0 关态：持久集合全量入图，_occ_seen 不参与 ----------------------------
s = make_self(pc_enabled=False)
s._occ_cells.add((3, 3))
s._occ_cells.add((4, 3))
s._occ_seen[(15, 15)] = _NOW[0]          # 即使开态才用的结构塞了新格也不该入图
H._rebuild_global_grid(s)
assert s._gocc_infl is not None and s._gocc_infl[3, 3], "C0: 持久格应在图"
assert not s._gocc_infl[15, 15], "C0: 关态不应消费 _occ_seen"
print("C0 PASS")

# ---- C1 开态：窗内保留，窗外衰减（剪枝） ------------------------------------
s = make_self(pc_enabled=True, ttl=45.0)
s._occ_seen[(3, 3)] = _NOW[0] - 10.0     # 窗内
s._occ_seen[(14, 4)] = _NOW[0] - 50.0    # 窗外（>45s 未重观测；远离窗内格膨胀域）
H._rebuild_global_grid(s)
assert s._gocc_infl[3, 3], "C1: 窗内格应在图"
assert not s._gocc_infl[4, 14], "C1: 窗外格不应在图"
assert (14, 4) not in s._occ_seen, "C1: 窗外格应被剪枝"
assert (3, 3) in s._occ_seen, "C1: 窗内格应保留"
print("C1 PASS")

# ---- C2 碰撞标记持久（不随窗衰减，且绕过足迹清除） ---------------------------
s = make_self(pc_enabled=True, ttl=45.0)
s._occ_seen[(3, 3)] = _NOW[0] - 100.0    # 旧 lidar 格：衰减
s._collision_obs.add((16, 5))            # 标记格（远处，不与 est 重叠）
H._rebuild_global_grid(s)
assert not s._gocc_infl[3, 3], "C2: 旧 lidar 格应衰减"
assert s._gocc_infl[5, 16] or s._gocc_infl[5, 17], "C2: 标记格应膨胀入图"
print("C2 PASS")

# ---- C3 自机足迹清除 + 膨胀 --------------------------------------------------
s = make_self(pc_enabled=True, ttl=45.0)
s._occ_seen[(10, 10)] = _NOW[0]          # est 所在格（新观测也必须被足迹清除）
s._occ_seen[(6, 6)] = _NOW[0]            # 正常占用格
H._rebuild_global_grid(s)
assert all(not s._gocc_infl[j, i]
           for j in (9, 10, 11) for i in (9, 10, 11)), "C3: est 3x3 应清空"
assert s._gocc_infl[7, 7], "C3: 占用格膨胀邻域应入图"
print("C3 PASS")

print("ALL SELFTEST PASS")
