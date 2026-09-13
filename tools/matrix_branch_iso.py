#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/matrix_branch_iso.py — 树枝场景逐机制隔离 A/B（归因工具）。

历史（20260912 首轮，run_logs/matrix_branch_iso_20260912_220622）：主矩阵
B_o1o5 0/3 PASS vs A_nobh 3/3 PASS——安全达成（col=0, minclr=0.87）但任务
完成归零，两形态：西墙滑扫（西树线 x≈-20.4 滑扫 ±10m 150s）+ 北界退赛
（y=17-18.8 悬停锁 OUT_OF_BOUNDS——geofence y≤15.35）。首轮归因结论：
  noO5（sway_margin=0）：stuck 71.9→10.8s 但 retired=5 不解 → O5=抖动加剧者
  noO3（反应半径/高度门回基线）：retired=5 不解 → O3 平反
  noO4：从未触发（无 UNC-SLOW 日志）→ 控制组
  all-minus（首轮补丁泄漏的意外产物）：retired=5 仍不解 → 根因=O1 判占线
首轮工具缺陷：变体键只切不复位 → 补丁跨 cell 累积（01 起=叠加切除）；
已修（每 cell 全量复位）。O5 已随判占线重设计退役（sway_margin 键删除），
noO5 臂随之移除。

现行臂（B 配置其余不动），seed 42+43：
  noO3  ca_react_extra=0 + ca_height_extra=0 → 反应半径/高度门回基线
  noO4  uncertain.enabled=false → 隔离 O4 限速

用法: python3 ~/zx2026_arena_ws/tools/matrix_branch_iso.py
"""
import glob
import os
import re
import shutil
import subprocess
import time

from matrix_run import (set_top, set_in_block, set_key_in_block,
                        parse_metrics, WS, CFG,
                        VERIFY_TXT, SMOKE_LOG, VERIFY_TIMEOUT)

CR = WS + "/src/zx2026_common/config/competition_rules.yaml"

CELLS = []
for seed in (42, 43):
    CELLS.append(dict(tag="B_noO3", seed=seed, patch="noO3"))
    CELLS.append(dict(tag="B_noO4", seed=seed, patch="noO4"))


def _patch_cell(cell):
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    set_top(CFG, "run_seed", cell["seed"])
    # 场景键（B 臂全开配置）
    set_in_block(CFG, "branches", "true")
    set_in_block(CFG, "branch_handling", "true")
    set_key_in_block(CR, "heights", "cruise_z", 1.8)
    # 变体键每 cell 全量复位再切除——首轮教训：只设本 cell 切除键不复位
    # 其他变体 → 补丁跨 cell 累积（01 起实为叠加切除）。归因结论侥幸成立
    # （all-minus 臂 retired=5 反证根因在 O1），工具必须显式复位。
    set_key_in_block(CFG, "branch_handling", "ca_react_extra", 0.6)
    set_key_in_block(CFG, "branch_handling", "ca_height_extra", 0.3)
    set_key_in_block(CFG, "uncertain", "enabled", "true")
    if cell["patch"] == "noO3":
        set_key_in_block(CFG, "branch_handling", "ca_react_extra", 0.0)
        set_key_in_block(CFG, "branch_handling", "ca_height_extra", 0.0)
    if cell["patch"] == "noO4":
        set_key_in_block(CFG, "uncertain", "enabled", "false")
    # 其余显式落盘提交默认（防跨 cell 泄漏）
    set_in_block(CFG, "temporal_consistency", "false")
    set_in_block(CFG, "wind", "false")
    set_in_block(CFG, "deadend_escape", "true")
    set_in_block(CFG, "veto_gate", "false")
    set_key_in_block(CFG, "point_cache", "enabled", "false")
    set_key_in_block(CFG, "drift_aware_margin", "enabled", "false")
    set_key_in_block(CFG, "near_stop_zone", "enabled", "true")
    set_key_in_block(CFG, "rescue_mutex", "enabled", "true")
    set_key_in_block(CFG, "bounce_corridor", "enabled", "true")
    set_key_in_block(CFG, "sep_obs_guard", "enabled", "true")
    set_key_in_block(CFG, "hotspot_inflate", "enabled", "false")
    set_key_in_block(CFG, "vo_avoid", "enabled", "false")
    set_key_in_block(CFG, "nav", "inflation", 0.4)


def _run_once(cell, outdir):
    _patch_cell(cell)
    t0 = time.time()
    m0 = time.monotonic()
    timed_out = False
    try:
        subprocess.run(["bash", WS + "/run_verify.sh"],
                       capture_output=True, text=True, timeout=VERIFY_TIMEOUT)
    except subprocess.TimeoutExpired:
        timed_out = True
        subprocess.run(["pkill", "-f", "rosmaster.*11411"], capture_output=True)
        subprocess.run(["pkill", "-f", "zx2026_all.launch"], capture_output=True)
        time.sleep(2)
    dt = time.time() - t0
    drift = dt - (time.monotonic() - m0)
    row = dict(tag=cell["tag"], seed=cell["seed"], patch=cell["patch"],
               verdict="FAIL", score=-1, done_t=-1, col=-1, stuck_s=-1,
               stuck_n=-1, flips=-1, dist=-1, minclr=-1,
               runtime_s=round(dt), timeout=timed_out, sleep_s=round(drift, 1))
    try:
        txt = open(VERIFY_TXT).read()
        if "VERDICT: PASS" in txt:
            row["verdict"] = "PASS"
        m = re.search(r"score_summary: total=(\d+)", txt)
        if m:
            row["score"] = int(m.group(1))
        m = re.search(r"-- state -> DONE @ (\d+)s", txt)
        if m:
            row["done_t"] = int(m.group(1))
        # 形态采样：北界退赛数 / 西墙滑扫（0/1 机 x 均值 ~ -20.4）
        m = re.search(r"report retired: \[([0-9, ]*)\]", txt)
        row["retired_n"] = len(m.group(1).replace(" ", "").split(",")) if m and m.group(1).strip() else 0
    except OSError:
        pass
    ymls = sorted(glob.glob(WS + "/run_logs/nav_metrics_2*.yaml"),
                  key=os.path.getmtime)
    if ymls:
        try:
            row.update(parse_metrics(ymls[-1]))
        except Exception as e:
            print("metrics parse fail:", e, flush=True)
    os.makedirs(outdir, exist_ok=True)
    for src, dst in ((VERIFY_TXT, "verify.txt"), (SMOKE_LOG, "smoke.log")):
        try:
            shutil.copy(src, os.path.join(outdir, dst))
        except OSError:
            pass
    if ymls:
        shutil.copy(ymls[-1], os.path.join(outdir, "nav_metrics.yaml"))
    return row, drift


def run_cell(cell, outdir):
    row, drift = _run_once(cell, outdir)
    if drift > 30.0:
        print("    !! wall-monotonic 漂移 %.0fs —— 留档 __contaminated，自动重跑"
              % drift, flush=True)
        shutil.rmtree(outdir + "__contaminated", ignore_errors=True)
        try:
            os.rename(outdir, outdir + "__contaminated")
        except OSError:
            pass
        row, drift = _run_once(cell, outdir)
    row["sleep_s"] = round(drift, 1)
    return row


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "matrix_branch_iso_" + ts)
    os.makedirs(outroot, exist_ok=True)
    bak_cfg = CFG + ".mbiso_bak"
    bak_cr = CR + ".mbiso_bak"
    shutil.copy(CFG, bak_cfg)
    shutil.copy(CR, bak_cr)
    csv_path = os.path.join(outroot, "iso_results.csv")
    cols = ["tag", "seed", "verdict", "score", "done_t", "retired_n", "col",
            "stuck_s", "flips", "dist", "minclr", "runtime_s", "sleep_s"]
    print("branch iso matrix start ->", outroot, flush=True)
    rows = []
    try:
        for k, cell in enumerate(CELLS):
            name = "%02d_%s_seed%d" % (k, cell["tag"], cell["seed"])
            print("[%d/%d] %s running..." % (k + 1, len(CELLS), name), flush=True)
            row = run_cell(cell, os.path.join(outroot, name))
            rows.append(row)
            with open(csv_path, "a") as f:
                if k == 0:
                    f.write(",".join(cols) + "\n")
                f.write(",".join(str(row[c]) for c in cols) + "\n")
            print("    -> %s score=%s done=%ss retired=%s col=%s stuck=%.1fs "
                  "(%.0fs)" % (row["verdict"], row["score"], row["done_t"],
                               row["retired_n"], row["col"], row["stuck_s"],
                               row["runtime_s"]), flush=True)
    finally:
        shutil.copy(bak_cfg, CFG)
        os.remove(bak_cfg)
        shutil.copy(bak_cr, CR)
        os.remove(bak_cr)
    print("\n===== SUMMARY (mean per variant) =====", flush=True)
    for tag in sorted({r["tag"] for r in rows}):
        arm = [r for r in rows if r["tag"] == tag]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%-7s n=%d pass=%d retired=%.0f score=%.1f done=%.0fs col=%.1f "
              "stuck=%.1fs minclr=%.3f"
              % (tag, n, passes, mean("retired_n"), mean("score"),
                 mean("done_t"), mean("col"), mean("stuck_s"), mean("minclr")),
              flush=True)
    print("results csv:", csv_path, flush=True)


if __name__ == "__main__":
    main()
