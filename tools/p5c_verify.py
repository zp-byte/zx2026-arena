#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p5c_verify.py — P5 退出迟滞同码 A/B（seed43 B 臂，归因 B43p3 FAIL）。

p5b 的 B43p3 FAIL（返航腿超时，全机仍在动、无 stall、de 早于窗口尾 4.5min
收敛）需要归因：P5 回归还是 seed43 B 臂固有方差（历史 B43 FAIL→B43r PASS
772.9s 累计 stuck，seed43 本就飘）。

同码 A/B：hyst=0 等价旧行为（单次规划成功即退出），hyst=2.5 为 P5。
  H0a-H0d  seed43 veto=on hyst=0   旧行为臂 ×4
对照已有 P5 臂（p5_/p5b_）：B43p/PASS、B43p2/PASS、B43p3/FAIL、B43p4/PASS = 3/4。
判据：若 hyst=0 臂同样出现 FAIL/更高 stuck，FAIL 归因固有方差，P5 无回归。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="H0a", seed=43, tc=False, wind=False, de=True, veto=True, hyst=0),
    dict(tag="H0b", seed=43, tc=False, wind=False, de=True, veto=True, hyst=0),
    dict(tag="H0c", seed=43, tc=False, wind=False, de=True, veto=True, hyst=0),
    dict(tag="H0d", seed=43, tc=False, wind=False, de=True, veto=True, hyst=0),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p5c_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p5c_bak"
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
    print("\n===== P5C SUMMARY (seed43 B-arm hyst A/B) =====", flush=True)
    print("-- hyst=0 (old behavior) arm:", flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs (%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["runtime_s"]), flush=True)
    print("-- hyst=2.5 (P5) arm (p5_/p5b_ archives):", flush=True)
    print("B43p      PASS score=154 done=160s col=0 stuck=33.5s")
    print("B43p2     PASS score=153 done=160s col=5 stuck=31.3s")
    print("B43p3     FAIL score=153 done=-1s  col=4  (return-leg window)")
    print("B43p4     PASS score=157 done=165s col=1 stuck=35.2s", flush=True)
    p0 = sum(1 for r in rows if r["verdict"] == "PASS")
    print("P5C: hyst=0 pass %d/4  vs  hyst=2.5 pass 3/4" % p0, flush=True)


if __name__ == "__main__":
    main()
