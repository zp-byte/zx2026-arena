#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p5b_verify.py — P5 seed43 多 run 补充（B43 染毒形态是随机事件）。

p5_verify 首轮 B43p 未复现 B43r 的投放点 flap 形态（col=0、de 零触发），
单 run 不能下"治好"结论（同 seed 跑次方差教训）。补 3 次 seed43 veto=on，
判据：任何一次出现 flap 形态（episode <2.5s 或 done 后 3s 内再 engage）
即迟滞未生效；stuck 全部有界（<100s）且全 PASS。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="B43p2", seed=43, tc=False, wind=False, de=True, veto=True),
    dict(tag="B43p3", seed=43, tc=False, wind=False, de=True, veto=True),
    dict(tag="B43p4", seed=43, tc=False, wind=False, de=True, veto=True),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p5b_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p5b_bak"
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
    print("\n===== P5B SUMMARY (seed43 x4 incl. first B43p) =====", flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs (%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["runtime_s"]), flush=True)
    print("P5B:", "ALL PASS" if all(r["verdict"] == "PASS" for r in rows)
          else "HAS FAIL", flush=True)


if __name__ == "__main__":
    main()
