#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/matrix_ei_settle.py — E/I 孤立地板销账矩阵（2026-09-15）。

背景：Wave B 收官后残余 0.7 col 法证定谳=梢尖（branch_tip t=1.00 端点）擦碰
（侵入仅 1-5mm，I 臂同梢同点 1cm 级复现 tree#25 br2）。孤立地板 ca_iso_floor
正为点状目标设计（孤立点 ≤3 判定 + push ≥2.0 抵巡航），但 I 臂旧基线未兑现
——本矩阵在 H 新基线（现行默认）上单旗检验其增量。

臂（相对现行默认只动一个键）：
  H_base   现行提交默认（lissajous=true + cloud_mem_frames=3，iso=0）
  J_hiso   H + ca_iso_floor=2.0

销账判据（预登记）：J 臂 3 seed col 合计 < H 且 score/done 无恶化 → iso 有
增量，留 pilot 再议翻默认；否则销账（保持默认关）。

交错：for seed in (42,43,44): for arm in 两臂——同 seed 臂对相邻。每 cell：
补丁→run_verify.sh→解析→归档 run_logs/matrix_ei_<ts>/。结束恢复两 yaml。
wall-monotonic 漂移检测继承 matrix_waveb_run.run_cell。

用法: python3 ~/zx2026_arena_ws/tools/matrix_ei_settle.py
"""
import os
import shutil
import subprocess
import time

import matrix_waveb_run as MW
from matrix_run import set_top, set_in_block, set_key_in_block, WS, CFG

CR = MW.CR

ARMS = ["H_base", "J_hiso"]
CELLS = [dict(tag=a, seed=s) for s in (42, 43, 44) for a in ARMS]


def _patch_cell_ei(cell):
    """两臂公共态=现行默认；仅 iso 分臂；其余显式落盘防跨 cell 泄漏。"""
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    set_top(CFG, "run_seed", cell["seed"])
    tag = cell["tag"]
    # ---- 场景/总开关：两臂同场景 ----
    set_in_block(CFG, "branches", "true")
    set_in_block(CFG, "branch_handling", "true")
    set_key_in_block(CR, "heights", "cruise_z", 1.8)
    # ---- H 公共态（现行默认）----
    set_key_in_block(CFG, "branch_handling", "contact_mark", "false")
    set_key_in_block(CFG, "branch_handling", "cloud_mem_frames", 3)
    set_key_in_block(CFG, "branch_handling", "ca_iso_floor",
                     2.0 if tag == "J_hiso" else 0.0)
    MW._set_map_z_band(CFG, "[1.45, 2.0]")
    set_key_in_block(CFG, "lidar", "lissajous", "true")
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


MW._patch_cell = _patch_cell_ei   # 复用 MW._run_once/_run_once 的补丁钩子


def main():
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "matrix_ei_" + ts)
    os.makedirs(outroot, exist_ok=True)
    bak_cfg = CFG + ".ei_bak"
    bak_cr = CR + ".ei_bak"
    shutil.copy(CFG, bak_cfg)
    shutil.copy(CR, bak_cr)
    csv_path = os.path.join(outroot, "matrix_results.csv")
    cols = ["tag", "seed", "verdict", "score", "done_t", "col", "stuck_s",
            "stuck_n", "flips", "dist", "minclr", "runtime_s", "sleep_s",
            "timeout"]
    print("ei settle matrix start ->", outroot, flush=True)
    rows = []
    try:
        for k, cell in enumerate(CELLS):
            name = "%02d_%s_seed%d" % (k, cell["tag"], cell["seed"])
            print("[%d/%d] %s running..." % (k + 1, len(CELLS), name),
                  flush=True)
            row = MW.run_cell(cell, os.path.join(outroot, name))
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
        print("%-7s n=%d pass=%d score=%.1f done=%.0fs col=%.2f stuck=%.1fs "
              "flips=%.1f dist=%.1f minclr=%.3f"
              % (tag, n, passes, mean("score"), mean("done_t"), mean("col"),
                 mean("stuck_s"), mean("flips"), mean("dist"),
                 mean("minclr")), flush=True)
    print("results csv:", csv_path, flush=True)


if __name__ == "__main__":
    main()
