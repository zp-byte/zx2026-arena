#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/matrix_branch_run.py — 树枝场景 A/B 矩阵（O1-O5 决策核心总验证）。

场景：branches.enabled=true（枝茂森林）+ competition_rules.yaml heights.cruise_z
2.5→1.8（穿越枝带；nav_node/mission_executor/task_generator 三读方同源单点）。

臂（交错执行，防时段漂移混淆臂间差异）：
  A_nobh   branches=on, branch_handling=off   现行逻辑（无 O1-O4）
  B_o1o5   branches=on, branch_handling=on    O1-O4（证据积分占用[判占线=
             密度线]/不确定减速/高度带/反应层；O5 风摆膨胀已随 20260912
             判占线重设计退役——见 sim_settings branch_handling 注释）
同 seed 下两臂枝几何逐位一致（br_rng 只依赖 forest_seed+run_seed，与开关无关）。

每 cell：补丁（只动值不动注释）→ bash run_verify.sh → 解析 /tmp/zx2026_verify.txt
+ 最新 run_logs/nav_metrics_*.yaml → 归档 run_logs/matrix_branch_<ts>/<tag>_seed<n>/。
结束恢复 sim_settings.yaml 与 competition_rules.yaml 原状。

wall-monotonic 漂移检测继承 matrix_run 纪律（>30s=睡眠污染→留档 __contaminated 自动重跑）。

用法: python3 ~/zx2026_arena_ws/tools/matrix_branch_run.py
"""
import glob
import os
import re
import shutil
import subprocess
import time

import yaml

from matrix_run import (set_top, set_in_block, set_key_in_block,
                        parse_metrics, WS, CFG,
                        VERIFY_TXT, SMOKE_LOG, VERIFY_TIMEOUT)

CR = WS + "/src/zx2026_common/config/competition_rules.yaml"

# 交错：A42 B42 A43 B43 A44 B44（同 seed 臂对相邻，时段漂移影响最小化）
CELLS = []
for seed in (42, 43, 44):
    CELLS.append(dict(tag="A_nobh", seed=seed, bh=False))
    CELLS.append(dict(tag="B_o1o5", seed=seed, bh=True))


def _patch_cell(cell):
    """补丁当前配置到本 cell 状态（只动值，不动注释；全部显式落盘防跨 cell 泄漏）。"""
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    set_top(CFG, "run_seed", cell["seed"])
    # ---- 场景键（两臂同场景，仅 branch_handling 分臂） ----
    set_in_block(CFG, "branches", "true")          # 场景主开关：枝茂森林
    set_in_block(CFG, "branch_handling",
                 "true" if cell["bh"] else "false")  # O1-O5 总开关
    set_key_in_block(CR, "heights", "cruise_z", 1.8)  # 穿越枝带（0.8-2.0m）
    # ---- 其余显式落盘为提交默认（防上一实验残留泄漏进本 cell） ----
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
    """跑一次 run_verify.sh 并收集结果；返回 (row, sleep_drift_s)。"""
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

    row = dict(tag=cell["tag"], seed=cell["seed"], verdict="FAIL", score=-1,
               done_t=-1, col=-1, stuck_s=-1, stuck_n=-1, flips=-1, dist=-1,
               minclr=-1, runtime_s=round(dt), timeout=timed_out,
               sleep_s=round(drift, 1))
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
    scores = [p for p in glob.glob("/tmp/zx2026_score_*.yaml")
              if os.path.getmtime(p) >= t0]   # 只认本 run 落盘的 report（防旧局串档）
    if scores:
        latest = max(scores, key=os.path.getmtime)
        shutil.copy(latest, os.path.join(outdir, "score.yaml"))
        # 权威分取封卷 report 的 total：score_summary 行会被末段 per-drone
        # 摘要覆盖（mission_done 后仍有迟到计分更新），total= 正则会落空
        # （matrix_branch_20260913_095951 六 cell 全部 score=-1 的根因）。
        # regex 只作 report 缺失时的兜底。
        try:
            row["score"] = int(yaml.safe_load(open(latest))["total"])
        except Exception as e:
            print("score.yaml parse fail:", e, flush=True)
    return row, drift


def run_cell(cell, outdir):
    """跑一个 cell；检出睡眠/断档污染则留档并自动重跑一次。"""
    row, drift = _run_once(cell, outdir)
    if drift > 30.0:
        print("    !! wall-monotonic 漂移 %.0fs —— 运行中断档/睡眠，cell 污染："
              "现场留档 __contaminated，自动重跑" % drift, flush=True)
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
    outroot = os.path.join(WS, "run_logs", "matrix_branch_" + ts)
    os.makedirs(outroot, exist_ok=True)
    bak_cfg = CFG + ".mbranch_bak"
    bak_cr = CR + ".mbranch_bak"
    shutil.copy(CFG, bak_cfg)
    shutil.copy(CR, bak_cr)
    csv_path = os.path.join(outroot, "matrix_results.csv")
    cols = ["tag", "seed", "verdict", "score", "done_t", "col", "stuck_s",
            "stuck_n", "flips", "dist", "minclr", "runtime_s", "sleep_s",
            "timeout"]
    print("branch matrix start ->", outroot, flush=True)
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
            print("    -> %s score=%s done=%ss col=%s stuck=%.1fs flips=%s "
                  "(%.0fs)" % (row["verdict"], row["score"], row["done_t"],
                               row["col"], row["stuck_s"], row["flips"],
                               row["runtime_s"]), flush=True)
    finally:
        shutil.copy(bak_cfg, CFG)
        os.remove(bak_cfg)
        shutil.copy(bak_cr, CR)
        os.remove(bak_cr)

    print("\n===== SUMMARY (mean per arm) =====", flush=True)
    for tag in sorted({r["tag"] for r in rows}):
        arm = [r for r in rows if r["tag"] == tag]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%-8s n=%d pass=%d score=%.1f done=%.0fs col=%.1f stuck=%.1fs "
              "flips=%.1f dist=%.1f minclr=%.3f"
              % (tag, n, passes, mean("score"), mean("done_t"), mean("col"),
                 mean("stuck_s"), mean("flips"), mean("dist"), mean("minclr")),
              flush=True)
    print("results csv:", csv_path, flush=True)


if __name__ == "__main__":
    main()
