#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/rescue_verify.py — 恢复链路仲裁修复对 A/B 验证（4 臂 × 3 seed）。

背景（2026-09-02 live 151 分 run 法证，tools/d5_forensic.py）：
  drone5：碰撞标记自封锁 → plan_fail 14.7-24.6s → de 触发（含 dir=None），
  GOAL-SEAL 盲推同窗口发射 7 次 → 双救援反向 → 研磨 + flip 风暴 → 6 碰撞 220s。
  drone3：±0.99 同 y 反向双撞——窄点两侧弹摆形态。
修复：M1 rescue_mutex（de 活跃/plan_fail 连续期间 goal-seal 禁发）
     M2 bounce_corridor（逃逸方向评分加连续走廊裕度，同可达偏好宽走廊）

矩阵（默认栈 de=on veto=off tc=off wind=off nstop=off dam=off infl=0.4）：
  O42/O43/O45    rm=F bc=F（现默认）
  M42/M43/M45    rm=T bc=F（单测互斥）
  B42/B43/B45    rm=F bc=T（单测走廊化）
  X42/X43/X45    rm=T bc=T（合臂）
判定：X 或 M 臂 col 相对 O 改善（同 seed 对 P<O）且 stuck/done 无回环
     → 翻默认；无改善或回环 → 保持关并记录重评条件。触发日志证据：
     M/X 臂 smoke.log 中 GOAL-SEAL 不应出现在 DEAD-END engage 之后 2s 内。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

BASE = dict(tc=False, wind=False, de=True, veto=False, dam=False,
            nstop=False, infl=0.4)

CELLS = []
for seed in (42, 43, 45):
    CELLS.append(dict(tag="O%d" % seed, seed=seed, rm=False, bc=False, **BASE))
for seed in (42, 43, 45):
    CELLS.append(dict(tag="M%d" % seed, seed=seed, rm=True, bc=False, **BASE))
for seed in (42, 43, 45):
    CELLS.append(dict(tag="B%d" % seed, seed=seed, rm=False, bc=True, **BASE))
for seed in (42, 43, 45):
    CELLS.append(dict(tag="X%d" % seed, seed=seed, rm=True, bc=True, **BASE))

ARMS = [("O", "rm=F bc=F(默认)"), ("M", "rm=T bc=F"), ("B", "rm=F bc=T"),
        ("X", "rm=T bc=T")]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "rescue_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".rescue_bak"
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
    print("\n===== RESCUE A/B SUMMARY =====", flush=True)
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
    print("\n同 seed 逐对 (col: O 为基准):", flush=True)
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
