#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/avoid_verify.py — 全史碰撞聚类驱动的避障优化对 A/B 验证（4 臂 × 3 seed）。

背景（2026-09-03 全史碰撞聚类，/tmp/col_analyze.py）：
  734 事件（python 706/131run + gazebo 28/6run）：Top4 树占 41%
  （tree#28 (-12.2,6.1) 145 次=20%）；主形态=分离硬推（障碍全盲）与云避障
  拔河 → 残余 + vcap 地板挤入；跟车同点连撞（d5+d3 相隔 3.4s 同点）与
  窄点双向撞（tree#24 双向 dir）均此。
修复：M3 sep_obs_guard（分离增量投影掉指向贴身障碍的分量，保切向）
     M4 hotspot_inflate（13 棵热点树格 +extra_cells 定向膨胀）

矩阵（现默认栈 rm=T bc=T de=T veto=off tc=off wind=off nstop=off dam=off infl=0.4）：
  X42/X43/X45    soa=F hi=F（现默认=rescue X 臂）
  S42/S43/S45    soa=T hi=F（单测分离投影）
  H42/H43/H45    soa=F hi=T（单测定向膨胀）
  D42/D43/D45    soa=T hi=T（合臂）
判定：S/H/D 臂 col 相对 X 改善（同 seed 对）且 stuck/minclr/done 无回环
     → 翻默认；无改善或回环 → 保持关并记录重评条件。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

BASE = dict(tc=False, wind=False, de=True, veto=False, dam=False,
            nstop=False, infl=0.4, rm=True, bc=True)

CELLS = []
for seed in (42, 43, 45):
    CELLS.append(dict(tag="X%d" % seed, seed=seed, soa=False, hi=False, **BASE))
for seed in (42, 43, 45):
    CELLS.append(dict(tag="S%d" % seed, seed=seed, soa=True, hi=False, **BASE))
for seed in (42, 43, 45):
    CELLS.append(dict(tag="H%d" % seed, seed=seed, soa=False, hi=True, **BASE))
for seed in (42, 43, 45):
    CELLS.append(dict(tag="D%d" % seed, seed=seed, soa=True, hi=True, **BASE))

ARMS = [("X", "soa=F hi=F(默认)"), ("S", "soa=T hi=F"),
        ("H", "soa=F hi=T"), ("D", "soa=T hi=T")]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "avoid_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".avoid_bak"
    shutil.copy(CFG, backup)
    rows = []
    try:
        for k, cell in enumerate(CELLS):
            name = "%d_%s" % (k + 1, cell["tag"])
            print("[%d/%d] %s running..." % (k + 1, len(CELLS), name), flush=True)
            row = run_cell(cell, os.path.join(outroot, name))
            rows.append(row)
            print("    -> %s score=%s done=%ss col=%s stuck=%.1fs flips=%s "
                  "minclr=%s (%.0fs sleep=%.0fs)"
                  % (row["verdict"], row["score"], row["done_t"], row["col"],
                     row["stuck_s"], row["flips"], row["minclr"],
                     row["runtime_s"], row.get("sleep_s", -1)), flush=True)
    finally:
        shutil.copy(backup, CFG)
        os.remove(backup)
    print("\n===== AVOID A/B SUMMARY =====", flush=True)
    for tag, desc in ARMS:
        arm = [r for r in rows if r["tag"].startswith(tag)]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%s臂(%s) n=%d pass=%d score=%.1f done=%.0fs col=%.2f "
              "stuck=%.1fs flips=%.1f minclr=%.3f"
              % (tag, desc, n, passes, mean("score"), mean("done_t"),
                 mean("col"), mean("stuck_s"), mean("flips"), mean("minclr")),
              flush=True)
    print("\n同 seed 逐对 (col: X 为基准):", flush=True)
    for seed in (42, 43, 45):
        line = "seed %d: " % seed
        for tag, _ in ARMS:
            r = next(x for x in rows if x["tag"] == "%s%d" % (tag, seed))
            line += "%s=%s/%ss  " % (tag, r["col"], r["done_t"])
        print(line, flush=True)
    for r in rows:
        print("%-6s %s score=%s done=%ss col=%s stuck=%.1fs flips=%s "
              "minclr=%s (%.0fs sleep=%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["flips"], r["minclr"], r["runtime_s"],
                 r.get("sleep_s", -1)), flush=True)


if __name__ == "__main__":
    main()
