#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_test_grid_acc.py — agent 累积栅格纯逻辑离线测（不碰 ROS 话题）。"""
import base64
import sys

sys.path.insert(0, "/home/ubuntu/zx2026_arena_ws/tools/gcs")
import gcs_agent  # noqa: E402  (顶部 import rospy 需 ROS noetic env)

A = gcs_agent.Agent

# z 带：z=0.1（地面回波）丢、z=4.5（树冠上）丢、带内两格
cells = A._cloud_cells([(0.2, 0.3, 1.0), (0.7, 0.3, 1.0),
                        (0.1, 0.1, 0.1), (0.1, 0.1, 4.5)])
assert cells == {(0, 0), (1, 0)}, cells

# 单帧栅格：2 宽 1 高全占用
g1 = A._acc_grid({(0, 0): 1, (1, 0): 1})
assert (g1["w"], g1["h"]) == (2, 1)
dec = base64.b64decode(g1["rle"])
assert list(dec) == [1, 2], list(dec)   # 占用×2（行翻转下同）

# 累积扩张：新帧格 (5,3) 并入 → bbox 6×4，占用共 3 格、其余未知
acc = {(0, 0): 1, (1, 0): 1}
acc.update({k: 1 for k in A._cloud_cells([(2.6, 1.7, 1.0)])})  # →(5,3)
g2 = A._acc_grid(acc)
assert (g2["w"], g2["h"]) == (6, 4), (g2["w"], g2["h"])
dec2 = base64.b64decode(g2["rle"])
cnt = sum(dec2[k + 1] for k in range(0, len(dec2), 2) if dec2[k] == 1)
assert cnt == 3, cnt
x0y0 = (g2["x0"], g2["y0"])
assert x0y0 == (0.0, 0.0), x0y0

# 空累积 → None（不发空图）
assert A._acc_grid({}) is None

print("AGENT_GRID_ACC_OK")
