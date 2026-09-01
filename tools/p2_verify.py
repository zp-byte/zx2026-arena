#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/p2_verify.py — P2 指令否决层验证（复用 matrix_run 基建）。

设计（DeFoP M1 几何安全监督移植）：对最终水平指令做刹车包络检查，只缩模不改向。
A/B 四臂（tc 均关=默认、de 均开=新默认，逐 seed 交织跑）：
  A 臂 calm veto=off（=P3 新基线）    B 臂 calm veto=on
  C 臂 wind  veto=off                D 臂 wind  veto=on（风场碰撞 13.5 均值处
                                                期望否决层收益最大）
判据：B vs A 不劣（score/done），col 不增；D vs C col 显著降。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    # calm A/B 逐 seed 交织
    dict(tag="A42", seed=42, tc=False, wind=False, de=True, veto=False),
    dict(tag="B42", seed=42, tc=False, wind=False, de=True, veto=True),
    dict(tag="A43", seed=43, tc=False, wind=False, de=True, veto=False),
    dict(tag="B43", seed=43, tc=False, wind=False, de=True, veto=True),
    dict(tag="A44", seed=44, tc=False, wind=False, de=True, veto=False),
    dict(tag="B44", seed=44, tc=False, wind=False, de=True, veto=True),
    dict(tag="A45", seed=45, tc=False, wind=False, de=True, veto=False),
    dict(tag="B45", seed=45, tc=False, wind=False, de=True, veto=True),
    dict(tag="A46", seed=46, tc=False, wind=False, de=True, veto=False),
    dict(tag="B46", seed=46, tc=False, wind=False, de=True, veto=True),
    # wind C/D 逐 seed 交织
    dict(tag="C42", seed=42, tc=False, wind=True, de=True, veto=False),
    dict(tag="D42", seed=42, tc=False, wind=True, de=True, veto=True),
    dict(tag="C43", seed=43, tc=False, wind=True, de=True, veto=False),
    dict(tag="D43", seed=43, tc=False, wind=True, de=True, veto=True),
]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "p2_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".p2_bak"
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
    print("\n===== P2 VERIFY SUMMARY =====", flush=True)
    for r in rows:
        print("%-9s %s score=%s done=%ss col=%s stuck=%.1fs (%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["runtime_s"]), flush=True)
    # 分臂均值（PASS-only 分数 + 全体碰撞）
    for arm, vetoes in (("calm", ("A", "B")), ("wind", ("C", "D"))):
        for vt in vetoes:
            rs = [r for r in rows if r["tag"].startswith(vt)]
            if not rs:
                continue
            sc = [int(r["score"]) for r in rs if r["verdict"] == "PASS"]
            col = sum(int(r["col"]) for r in rs)
            done = [int(r["done_t"]) for r in rs if r["done_t"] != "-"]
            print("%s%c: n=%d PASS均分=%.1f col合计=%d DONE均=%.0fs"
                  % (arm, vt, len(rs),
                     sum(sc) / len(sc) if sc else float("nan"),
                     col, sum(done) / len(done) if done else float("nan")),
                  flush=True)
    ok = all(r["verdict"] == "PASS" for r in rows)
    print("P2 VERIFY:", "ALL PASS" if ok else "HAS FAIL", flush=True)


if __name__ == "__main__":
    main()
