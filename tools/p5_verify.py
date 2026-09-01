#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p5_verify.py — P5 de 退出迟滞验证（复用 matrix_run 基建）。

B43r 残留形态（run_logs/p2b_*）：drone5 投放点旁云推力平衡 + 规划逐拍翻转，
de 两进两出（1.6s/0.5s），STALL diag stuck 772.9s 仍 DONE。

修复：_de_tick 退出迟滞——规划【连续】稳定 exit_hyst_s(2.5) 才交还路径跟随，
迟滞期间继续沿逃逸方向（云走廊已验证）驶出，把自机推离标记/树格边缘。

本矩阵（de 迟滞随默认带出，veto 仍关）：
  B43p  seed43 veto=on   染毒 cell：主判据 stuck 772.9→? 且 PASS
  A43p  seed43 veto=off  同 seed 基线臂对照（A43r 156/165s）
  B42p  seed42 veto=on   干净 seed 回归 spot（B42r 157/150 col1 stuck14.9）
  D43p  seed43 wind veto=on 风场回归 spot（D43r PASS 残留磨树 2）
判定：4 cell 全 PASS + B43p stuck 显著回落 + STALL/escape 日志证据。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="B43p", seed=43, tc=False, wind=False, de=True, veto=True),
    dict(tag="A43p", seed=43, tc=False, wind=False, de=True, veto=False),
    dict(tag="B42p", seed=42, tc=False, wind=False, de=True, veto=True),
    dict(tag="D43p", seed=43, tc=False, wind=True, de=True, veto=True),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p5_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p5_bak"
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
    print("\n===== P5 VERIFY SUMMARY =====", flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs (%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["runtime_s"]), flush=True)
    all_ok = all(r["verdict"] == "PASS" for r in rows)
    print("P5 VERIFY:", "ALL PASS" if all_ok else "HAS FAIL", flush=True)


if __name__ == "__main__":
    main()
