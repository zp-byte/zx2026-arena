#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/dam_verify.py — 漂移感知裕度（dam）A/B 验证（同码，drift_aware_margin.enabled 0/1）。

背景（2026-08-31 live 158 分 run 碰撞法证）：三次接触全部低速贴树挤入
（接触时 0.12-0.98 m/s），机制 = est 系控制误差（漂移背向最近障碍时
裕度高估 |drift·u|）+ vcap 0.35 速度地板贴脸不封零。dam：vcap 用有效
裕度 eff=clearance−max(0,−drift·u)，地板随侵蚀占比衰减到 0。

预期收益：min_clear 抬升、col 下降。风险：近障段限速更紧 → done 变慢、
stuck/flips 变化；地板衰减在窄走廊可能拖慢穿越（eff<0.5 段 vcap<0.4）。

矩阵（默认栈 de=on veto=off tc=off wind=off）：
  O42/O43/O45  dam=off（现默认）
  P42/P43/P45  dam=on
判定：col/minclr 双臂对比 + PASS 率/score/done/stuck/flips 无回归。
注意：长 sweep 保持机器唤醒（睡眠会烧墙钟窗口，matrix_run 已有漂移
检测+自动重跑兜底）。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="O42", seed=42, tc=False, wind=False, de=True, veto=False, dam=False),
    dict(tag="O43", seed=43, tc=False, wind=False, de=True, veto=False, dam=False),
    dict(tag="O45", seed=45, tc=False, wind=False, de=True, veto=False, dam=False),
    dict(tag="P42", seed=42, tc=False, wind=False, de=True, veto=False, dam=True),
    dict(tag="P43", seed=43, tc=False, wind=False, de=True, veto=False, dam=True),
    dict(tag="P45", seed=45, tc=False, wind=False, de=True, veto=False, dam=True),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "dam_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".dam_bak"
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
    print("\n===== DAM A/B SUMMARY =====", flush=True)
    for tag in ("O", "P"):
        arm = [r for r in rows if r["tag"].startswith(tag)]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%s臂(dam=%s) n=%d pass=%d score=%.1f done=%.0fs col=%.2f "
              "stuck=%.1fs flips=%.1f minclr=%.3f"
              % (tag, tag == "P", n, passes, mean("score"), mean("done_t"),
                 mean("col"), mean("stuck_s"), mean("flips"), mean("minclr")),
              flush=True)
    for r in rows:
        print("%-6s %s score=%s done=%ss col=%s stuck=%.1fs flips=%s "
              "minclr=%s (%.0fs sleep=%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["flips"], r["minclr"], r["runtime_s"],
                 r.get("sleep_s", -1)), flush=True)


if __name__ == "__main__":
    main()
