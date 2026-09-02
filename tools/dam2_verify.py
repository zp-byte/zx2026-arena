#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/dam2_verify.py — dam v2（深近区门控 deep_r=0.7）A/B 验证。

v1 定论（run_logs/dam_20260831_181122）：col 4.0→2.67 有改善，但 P42 stuck
42→238s、P45 满窗 FAIL——持续向量漂移使整条腿地板归零，任务级减速。
v2：地板衰减收缩到 eff<0.7 贴脸带（基线地板 bind 边界），带外逐位回基线，
任务级减速结构性排除。触发：2026-09-02 live 复盘 10 次碰撞全为低速贴树挤入。

矩阵（默认栈 de=on veto=off tc=off wind=off nstop=off）：
  O42/O43/O45  dam=off（现默认）
  P42/P43/P45  dam=on（v2, deep_r=0.7）
判定：col 改善保留（P < O）且 无 P42-style stuck 爆炸、无满窗 FAIL
     → 翻默认；任一回环出现 → 保持关并记录重评条件。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="O42", seed=42, tc=False, wind=False, de=True, veto=False,
         dam=False, nstop=False, infl=0.4),
    dict(tag="O43", seed=43, tc=False, wind=False, de=True, veto=False,
         dam=False, nstop=False, infl=0.4),
    dict(tag="O45", seed=45, tc=False, wind=False, de=True, veto=False,
         dam=False, nstop=False, infl=0.4),
    dict(tag="P42", seed=42, tc=False, wind=False, de=True, veto=False,
         dam=True, nstop=False, infl=0.4),
    dict(tag="P43", seed=43, tc=False, wind=False, de=True, veto=False,
         dam=True, nstop=False, infl=0.4),
    dict(tag="P45", seed=45, tc=False, wind=False, de=True, veto=False,
         dam=True, nstop=False, infl=0.4),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "dam2_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".dam2_bak"
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
    print("\n===== DAM2 A/B SUMMARY =====", flush=True)
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
