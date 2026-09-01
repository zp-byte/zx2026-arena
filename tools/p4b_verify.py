#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p4b_verify.py — P4 A/B 补跑：p4 sweep 中跨机器休眠的 O45/O42。

p4 sweep 期间机器休眠过（O 臂 runtime 被 wall 时钟撑大 10 倍），ROS 传输层
跨休眠可能劣化（O45 出现 drone5 卡 EXECUTE 未投放的传输中断症状形态），
两个 FAIL 数据不可信。本脚本在唤醒状态下干净重跑同 cell。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="O45b", seed=45, tc=False, wind=False, de=True, veto=False, pc=False),
    dict(tag="O42b", seed=42, tc=False, wind=False, de=True, veto=False, pc=False),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p4b_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p4b_bak"
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
    print("\n===== P4B SUMMARY =====", flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs flips=%s (%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["flips"], r["runtime_s"]), flush=True)


if __name__ == "__main__":
    main()
