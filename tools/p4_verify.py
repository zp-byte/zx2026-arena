#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p4_verify.py — P4 滑窗点缓存 A/B（同码，point_cache.enabled 0/1）。

机制：占用格时间戳化，重观测刷新、超 ttl_s(45s) 衰减（重建时惰性剪枝）；
碰撞标记独立持久。预期收益：噪声/漂移错位格不再永生 → 树周涂抹变薄 →
封锁带变窄 → de/gs 触发率降、stuck 降。风险：远处真树被遗忘 → 路径切
换 churn（flips/done 上升）。

矩阵（de=on veto=off 当前默认栈）：
  O43/O45/O42  point_cache=off（现默认）seed 43/45/42
  P43/P45/P42  point_cache=on  同 seed 对照
判定：PASS 率/col/stuck/flips/done 双臂对比 + de/gs 触发率（机制指标，
跑后从 rosout 统计）。无显著差异 → 默认保持关（同 P1/P2 结论口径）。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="O43", seed=43, tc=False, wind=False, de=True, veto=False, pc=False),
    dict(tag="O45", seed=45, tc=False, wind=False, de=True, veto=False, pc=False),
    dict(tag="O42", seed=42, tc=False, wind=False, de=True, veto=False, pc=False),
    dict(tag="P43", seed=43, tc=False, wind=False, de=True, veto=False, pc=True),
    dict(tag="P45", seed=45, tc=False, wind=False, de=True, veto=False, pc=True),
    dict(tag="P42", seed=42, tc=False, wind=False, de=True, veto=False, pc=True),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p4_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p4_bak"
    shutil.copy(CFG, backup)
    rows = []
    try:
        for k, cell in enumerate(CELLS):
            name = "%d_%s" % (k + 1, cell["tag"])
            print("[%d/%d] %s running..." % (k + 1, len(CELLS), name), flush=True)
            row = run_cell(cell, os.path.join(outroot, name))
            rows.append(row)
            print("    -> %s score=%s done=%ss col=%s stuck=%.1fs flips=%s (%.0fs)"
                  % (row["verdict"], row["score"], row["done_t"], row["col"],
                     row["stuck_s"], row["flips"], row["runtime_s"]), flush=True)
    finally:
        shutil.copy(backup, CFG)
        os.remove(backup)
    print("\n===== P4 A/B SUMMARY =====", flush=True)
    for tag in ("O", "P"):
        arm = [r for r in rows if r["tag"].startswith(tag)]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%s臂(pc=%s) n=%d pass=%d score=%.1f done=%.0fs col=%.1f "
              "stuck=%.1fs flips=%.1f"
              % (tag, tag == "P", n, passes, mean("score"), mean("done_t"),
                 mean("col"), mean("stuck_s"), mean("flips")), flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs flips=%s (%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["flips"], r["runtime_s"]), flush=True)


if __name__ == "__main__":
    main()
