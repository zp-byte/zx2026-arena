#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p5d_verify.py — P5 真·同码 A/B：hyst=0（旧行为）×4 seed43 B 臂。

【p5c_ 作废】：set_key_in_block 的 ^ 锚点因块内缩进永远失配，p5c 四个
"hyst=0" cell 实际仍跑 exit_hyst_s=2.5（旁证：episode 全部 4.0-6.8s），
其数据并入 P5 臂（seed43 ×8 = 6/8 PASS）。本脚本在补丁修复后重跑。

对照 P5 臂（hyst=2.5）：B43p/PASS 154、B43p2/PASS 153、B43p3/FAIL（返航
超时）、B43p4/PASS 157、H0a-d（实为 2.5）PASS/PASS/FAIL/PASS。
判据：hyst=0 臂 FAIL 率与 stuck 水平 vs P5 臂，归因 B43p3 类 FAIL。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="G0a", seed=43, tc=False, wind=False, de=True, veto=True, hyst=0),
    dict(tag="G0b", seed=43, tc=False, wind=False, de=True, veto=True, hyst=0),
    dict(tag="G0c", seed=43, tc=False, wind=False, de=True, veto=True, hyst=0),
    dict(tag="G0d", seed=43, tc=False, wind=False, de=True, veto=True, hyst=0),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p5d_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p5d_bak"
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
    print("\n===== P5D SUMMARY (seed43 B-arm hyst=0 old-behavior arm) =====", flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs (%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["runtime_s"]), flush=True)
    p0 = sum(1 for r in rows if r["verdict"] == "PASS")
    print("P5D: hyst=0 pass %d/4  vs  hyst=2.5 (p5+p5b+p5c) 6/8" % p0, flush=True)


if __name__ == "__main__":
    main()
