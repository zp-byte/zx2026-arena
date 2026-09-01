#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p3_verify.py — P3 死端逃逸验证（复用 matrix_run 基建）。

第一轮（2026-08-30，run_logs/p3_20260830_182514）：
  R1 tc=on seed 42 PASS 152@170s（起点侧封锁逃逸生效，drone4 死锁 4.4s 脱困）
  R2/R4 FAIL → 诊断出第二形态：目标侧封锁（投放点膨胀环封死 → 最近可达格
  停车），零碰撞也发生，基线固有。→ 增加 P3b goal-seal 直达逼近。
第二轮（本脚本现 CELLS）：复跑 R2/R3/R4 验证 P3b。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="S45", seed=45, tc=False, wind=False, de=True),
    dict(tag="S46", seed=46, tc=False, wind=False, de=True),
    dict(tag="S47", seed=47, tc=False, wind=False, de=True),
    dict(tag="S48", seed=48, tc=False, wind=False, de=True),
    dict(tag="S49", seed=49, tc=False, wind=False, de=True),
    dict(tag="S50", seed=50, tc=False, wind=False, de=True),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p3_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p3_bak"
    shutil.copy(CFG, backup)
    rows = []
    try:
        for k, cell in enumerate(CELLS):
            name = "%d_%s" % (k + 1, cell["tag"])
            print("[%d/%d] %s running..." % (k + 1, len(CELLS), name), flush=True)
            row = run_cell(cell, os.path.join(outroot, name))
            rows.append(row)
            print("    -> %s score=%s done=%ss col=%s stuck=%.1fs (%.0fs)"
                  % (row["verdict"], row["score"], row["done_t"], row["col"],
                     row["stuck_s"], row["runtime_s"]), flush=True)
    finally:
        shutil.copy(backup, CFG)
        os.remove(backup)
    print("\n===== P3 VERIFY SUMMARY =====", flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs flips=%s"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["flips"]), flush=True)
    ok = all(r["verdict"] == "PASS" for r in rows)
    print("P3 VERIFY:", "ALL PASS" if ok else "HAS FAIL", flush=True)


if __name__ == "__main__":
    main()
