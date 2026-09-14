#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/matrix_waveb_run.py — Wave B 单旗 A/B 矩阵（2026-09-14）。

场景与 matrix_branch_run 同（branches=on + branch_handling=on，cruise 1.8）。
臂（每臂相对 B_base 只动一个键；同 seed 枝几何逐位一致）：
  B_base    现行提交态（contact/cloud_mem/iso_floor 全关，band [1.45,2.0]）
  C_contact  B + contact_mark=true              ——接触域标记（杀同梢复碰 67%）
  D_cloud    B + cloud_mem_frames=3             ——云缓存 2-3 帧梢记忆（杀盲帧 33%）
  E_iso      B + ca_iso_floor=2.0               ——孤立点斥力地板（杀 10-15%）
  F_cruise   B + cruise_z=1.95 + band [1.45,2.2] ——巡航抬升实验（需抬带顶）
  G_lis      B + sensor.lidar.lissajous=true     ——花式扫描（恒盲梢→闪烁可见，O1 新输入）
  H_clis     G + cloud_mem_frames=3              ——云记忆对可见梢的激活验证（梢点进记忆）
  I_ilis     G + ca_iso_floor=2.0                ——孤立地板对可见梢的激活验证（梢点可推）

交错：for seed in (42,43,44): for arm in 八臂 —— 同 seed 臂对相邻，时段漂移最小。
每 cell：补丁（只动值不动注释）→ run_verify.sh → 解析 → 归档 run_logs/matrix_waveb_<ts>/。
结束恢复两 yaml 原状。wall-monotonic 漂移检测继承 matrix_run 纪律（>30s 污染重跑）。

用法: python3 ~/zx2026_arena_ws/tools/matrix_waveb_run.py
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

ARMS = ["B_base", "C_contact", "D_cloud", "E_iso", "F_cruise",
        "G_lis", "H_clis", "I_ilis"]
CELLS = []
for seed in (42, 43, 44):
    for arm in ARMS:
        CELLS.append(dict(tag=arm, seed=seed))


def _set_map_z_band(path, value):
    """map_z_band: [1.45, 2.0] 整值替换（值含空格，set_key_in_block 的 \S+ 会截断）。"""
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    for i, ln in enumerate(lines):
        if re.match(r"^\s*map_z_band\s*:", ln):
            lines[i] = re.sub(r"^(\s*map_z_band\s*:\s*)\[.*\]",
                              lambda m: m.group(1) + value, ln, count=1)
            break
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)


def _patch_cell(cell):
    """补丁当前配置到本 cell 状态（只动值，不动注释；全部显式落盘防跨 cell 泄漏）。"""
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    set_top(CFG, "run_seed", cell["seed"])
    tag = cell["tag"]
    # ---- 场景/总开关：全部臂同场景，仅 Wave B 键分臂 ----
    set_in_block(CFG, "branches", "true")
    set_in_block(CFG, "branch_handling", "true")
    set_key_in_block(CR, "heights", "cruise_z",
                     1.95 if tag == "F_cruise" else 1.8)
    # ---- Wave B 键（每 cell 显式落盘默认，防泄漏） ----
    set_key_in_block(CFG, "branch_handling", "contact_mark",
                     "true" if tag == "C_contact" else "false")
    set_key_in_block(CFG, "branch_handling", "cloud_mem_frames",
                     3 if tag in ("D_cloud", "H_clis") else 0)
    set_key_in_block(CFG, "branch_handling", "ca_iso_floor",
                     2.0 if tag in ("E_iso", "I_ilis") else 0.0)
    _set_map_z_band(CFG, "[1.45, 2.2]" if tag == "F_cruise" else "[1.45, 2.0]")
    # ---- 传感保真度（G/H/I 臂共享 lissajous 翻转；其余臂显式落盘默认防泄漏） ----
    set_key_in_block(CFG, "lidar", "lissajous",
                     "true" if tag in ("G_lis", "H_clis", "I_ilis") else "false")
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
    outroot = os.path.join(WS, "run_logs", "matrix_waveb_" + ts)
    os.makedirs(outroot, exist_ok=True)
    bak_cfg = CFG + ".mwaveb_bak"
    bak_cr = CR + ".mwaveb_bak"
    shutil.copy(CFG, bak_cfg)
    shutil.copy(CR, bak_cr)
    csv_path = os.path.join(outroot, "matrix_results.csv")
    cols = ["tag", "seed", "verdict", "score", "done_t", "col", "stuck_s",
            "stuck_n", "flips", "dist", "minclr", "runtime_s", "sleep_s",
            "timeout"]
    print("waveb matrix start ->", outroot, flush=True)
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
    for tag in ARMS:
        arm = [r for r in rows if r["tag"] == tag]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%-9s n=%d pass=%d score=%.1f done=%.0fs col=%.1f stuck=%.1fs "
              "flips=%.1f dist=%.1f minclr=%.3f"
              % (tag, n, passes, mean("score"), mean("done_t"), mean("col"),
                 mean("stuck_s"), mean("flips"), mean("dist"), mean("minclr")),
              flush=True)
    print("results csv:", csv_path, flush=True)


if __name__ == "__main__":
    main()
