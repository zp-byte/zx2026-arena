#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/batch1_verify.py — 第一批优化 A/B：膨胀裕度（nav.inflation）+ 近停区（near_stop_zone）。

背景（2026-08-31 碰撞法证）：live 158 分 run 三次碰撞全为低速贴树挤入
（接触时 0.12-0.98 m/s）。（a）作业净空 0.32-0.5m 在花定位误差预算 →
膨胀裕度 0.4→0.9（栅格 ceil(0.75/0.5)=2 格 → ceil(1.25/0.5)=3 格，已核算
真实多出整整一格）；（b）vcap 地板 0.35 贴脸仍保底 0.53m/s → 近停区在
goal 前 zone_r=1.0m 线性衰减地板（deadband 0.2 归零），de 逃逸不衰减。

矩阵（默认栈 de=on veto=off tc=off wind=off dam=off，每 cell 无条件落盘
infl/nstop 防补丁泄漏）：
  m04_42/43/45  inflation=0.4（现默认）
  m09_42/43/45  inflation=0.9
  ns0_42/43/45  near_stop_zone=off（现默认）
  ns1_42/43/45  near_stop_zone=on
判定：m09 vs m04、ns1 vs ns0 —— col/minclr 改善且 done/stuck/flips/PASS 无回归。
注意：长 sweep 保持机器唤醒（matrix_run 有睡眠检测+自动重跑兜底）。
"""
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from matrix_run import CFG, WS, run_cell  # noqa: E402

CELLS = [
    dict(tag="m04_42", seed=42, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=False),
    dict(tag="m04_43", seed=43, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=False),
    dict(tag="m04_45", seed=45, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=False),
    dict(tag="m09_42", seed=42, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.9, nstop=False),
    dict(tag="m09_43", seed=43, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.9, nstop=False),
    dict(tag="m09_45", seed=45, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.9, nstop=False),
    dict(tag="ns0_42", seed=42, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=False),
    dict(tag="ns0_43", seed=43, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=False),
    dict(tag="ns0_45", seed=45, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=False),
    dict(tag="ns1_42", seed=42, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=True),
    dict(tag="ns1_43", seed=43, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=True),
    dict(tag="ns1_45", seed=45, tc=False, wind=False, de=True, veto=False,
         dam=False, infl=0.4, nstop=True),
]

ARMS = [("m04", "膨胀0.4(默认)"), ("m09", "膨胀0.9"),
        ("ns0", "近停关(默认)"), ("ns1", "近停开")]


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "batch1_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".batch1_bak"
    shutil.copy(CFG, backup)
    rows = []
    try:
        for k, cell in enumerate(CELLS):
            name = "%02d_%s" % (k + 1, cell["tag"])
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
    print("\n===== BATCH1 A/B SUMMARY =====", flush=True)
    for prefix, label in ARMS:
        arm = [r for r in rows if r["tag"].startswith(prefix)]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%-4s %-12s n=%d pass=%d score=%.1f done=%.0fs col=%.2f "
              "stuck=%.1fs flips=%.1f minclr=%.3f"
              % (prefix, label, n, passes, mean("score"), mean("done_t"),
                 mean("col"), mean("stuck_s"), mean("flips"), mean("minclr")),
              flush=True)
    for r in rows:
        print("%-8s %s score=%s done=%ss col=%s stuck=%.1fs flips=%s "
              "minclr=%s (%.0fs sleep=%.0fs)"
              % (r["tag"], r["verdict"], r["score"], r["done_t"], r["col"],
                 r["stuck_s"], r["flips"], r["minclr"], r["runtime_s"],
                 r.get("sleep_s", -1)), flush=True)


if __name__ == "__main__":
    main()
