#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""matrix_w1b 有碰撞 cell 的碰撞行提取（去重 raw/rosout 双份）。"""
import glob
import os
import re

D = os.path.expanduser("~/zx2026_arena_ws/run_logs/matrix_w1_20260903_203004")
for c in sorted(os.listdir(D)):
    p = os.path.join(D, c, "evidence.txt")
    if not os.path.exists(p):
        continue
    txt = open(p, encoding="utf-8", errors="replace").read()
    lines = sorted(set(re.findall(
        r"nav_node: drone \d collision marker at \([^)]*\) cell \([^)]*\) "
        r"close=[-\d.]+ clr=[-\d.]+", txt)))
    if not lines:
        continue
    print("== %s (%d) ==" % (c, len(lines)))
    for ln in lines:
        print("  " + ln)
