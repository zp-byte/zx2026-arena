#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p2b_verify.py — P2 修复包验证（复用 matrix_run 基建）。

首轮矩阵（run_logs/p2_20260830_202630）结论：veto 按原参数化净有害——
B 臂 highZ(≥3.4m) 磨树级联 40 次 vs A 臂 0（veto scale→0 冻结弹起后水平
脱离）+ B43 drone4 逃逸指令被 veto 冻结成 6 连撞 + B44 stuck 142s。

修复包（四项，2026-08-30）：
  1. _de_pick_dir 点云走廊检查（cloud_clear_r=1.0）：逃逸方向沿线上有点云
     即淘汰——栅格壳可穿≠真树可穿
  2. veto scale_min=0.35 地板：减速不冻结水平进度（对齐 vcap 地板）
  3. 末次有效路径保持：规划失败不清 _gpath，de 触发改用规划失败连续时长
     （治 flap 停滞）
  4. 停滞看门狗：>2s 近零速且距 goal>1m → 1Hz STALL diag 日志

本脚本重跑全部染毒/退化 cell + A43（非 veto 依赖形态，验证 3/4 项）：
  B42r-B46r  veto=on calm 全臂（对照首轮 A 臂：FAIL→0、highZ→0、stuck 回落）
  A43r       veto=off seed43（看门狗应给出 return 停滞形态证据）
  D43r       wind veto=on（对照 C43 col=7，查 wind 臂回归）
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="B42r", seed=42, tc=False, wind=False, de=True, veto=True),
    dict(tag="B43r", seed=43, tc=False, wind=False, de=True, veto=True),
    dict(tag="B44r", seed=44, tc=False, wind=False, de=True, veto=True),
    dict(tag="B45r", seed=45, tc=False, wind=False, de=True, veto=True),
    dict(tag="B46r", seed=46, tc=False, wind=False, de=True, veto=True),
    dict(tag="A43r", seed=43, tc=False, wind=False, de=True, veto=False),
    dict(tag="D43r", seed=43, tc=False, wind=True, de=True, veto=True),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p2b_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p2b_bak"
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
    print("\n===== P2B VERIFY SUMMARY =====", flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs (%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["runtime_s"]), flush=True)
    b_ok = all(r["verdict"] == "PASS" for r in rows if r["tag"].startswith("B"))
    print("P2B VERIFY:", "B-ARM ALL PASS" if b_ok else "B-ARM HAS FAIL", flush=True)


if __name__ == "__main__":
    main()
