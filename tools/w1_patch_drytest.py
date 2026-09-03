#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""W1 补丁干跑测试：patch_cell 落盘语义断言（不跑 sim）。"""
import shutil
import sys

import yaml

WS = "/home/ubuntu/zx2026_arena_ws"
sys.path.insert(0, WS + "/tools")
import w1_matrix_run as W

REAL = WS + "/src/zx2026_common/config/sim_settings.yaml"
TMP = "/tmp/w1_drytest_settings.yaml"


def state():
    d = yaml.safe_load(open(TMP))
    return d


def check(cond, msg):
    if not cond:
        raise AssertionError("DRYTEST FAIL: %s" % msg)
    print("  OK: %s" % msg)


shutil.copy(REAL, TMP)
W.CFG = TMP

# A_base 形态
W.patch_cell(W.CELLS[0])
s = state()
check(s["run_seed"] == 42, "A run_seed=42")
check(s["mission"]["via_slots"]["enabled"] is False, "A via_slots off")
check(s["swarm"]["rescue_quiet"]["enabled"] is False, "A rescue_quiet off")
check(s["swarm"]["obs_guard"]["enabled"] is False, "A obs_guard off")
check(s["closed_loop"]["sep_obs_guard"]["enabled"] is True, "A sep_obs_guard ON (基线栈)")
check(s["closed_loop"]["rescue_mutex"]["enabled"] is True, "A rescue_mutex ON")
check(s["closed_loop"]["bounce_corridor"]["enabled"] is True, "A bounce_corridor ON")
check(s["closed_loop"]["deadend_escape"]["enabled"] is True, "A deadend_escape ON")
check(float(s["closed_loop"]["deadend_escape"]["exit_hyst_s"]) == 2.5, "A exit_hyst=2.5")
for blk in ("veto_gate", "point_cache", "drift_aware_margin", "near_stop_zone",
            "temporal_consistency", "hotspot_inflate"):
    check(s["closed_loop"][blk]["enabled"] is False, "A %s off" % blk)
check(s["wind"]["enabled"] is False, "A wind off")
check(s["comms"]["enabled"] is False, "A comms off")
check(float(s["nav"]["inflation"]) == 0.4, "A inflation=0.4")
check(s["swarm"]["enabled"] is True, "A swarm ON (基线)")

# D_full 形态
W.patch_cell(W.CELLS[-1])
s = state()
check(s["run_seed"] == 45, "D run_seed=45")
check(s["mission"]["via_slots"]["enabled"] is True, "D via_slots ON")
check(s["swarm"]["rescue_quiet"]["enabled"] is True, "D rescue_quiet ON")
check(s["swarm"]["obs_guard"]["enabled"] is True, "D obs_guard ON")
check(s["closed_loop"]["sep_obs_guard"]["enabled"] is True, "D sep_obs_guard ON")

# 注释保留抽查（补丁只动值不动注释行）
orig = open(REAL, encoding="utf-8").read().splitlines()
new = open(TMP, encoding="utf-8").read().splitlines()
check(len(orig) == len(new), "line count unchanged (%d)" % len(new))
diff_lines = [(i, a, b) for i, (a, b) in enumerate(zip(orig, new)) if a != b]
print("  changed lines: %d" % len(diff_lines))
for i, a, b in diff_lines[:14]:
    print("    L%d: %r -> %r" % (i + 1, a.strip()[:46], b.strip()[:46]))
n_comment_only = sum(1 for _, a, b in diff_lines
                     if a.split("#")[0].rstrip() == b.split("#")[0].rstrip())
check(n_comment_only == 0, "no comment-only changes")

print("DRYTEST ALL PASS")
