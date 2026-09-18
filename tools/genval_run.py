#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/genval_run.py — ③ 新 seed 泛化验证（2026-09-18，② 收口后续）。

动机：W1/W2/② 三轮矩阵全部定谳于 seed{42,43,45}——机制是否对这三个 seed
的枝形/任务分配过拟合，抽 5 个新 seed 验证。

口径（先说清测什么）：
  - run_seed 变 = 枝形（br_rng=森林种子×run_seed）+ 任务分配 + 时序；
  - 树布局固定（density_seed=42，同森林同热点树编号）——"新森林"泛化
    须动 density_seed，另立案不在本轮。
  - seed47 特意选入：W6 时代三体回归 seed，压力样本。

栈 = 现默认全显式落盘（W1 三旗 + W2-P2 boundary_margin + ② mem6），
单臂无 A/B；seed{46,47,48,49,50} × 2 遍 = 10 cells。

毙杀口径（预登记）：
  - 任一 FAIL → 法证归因（OOD seed 上机制失效？）；
  - Σcol/run 均值显著超基线（B 臂近期 ~1.2 起/run）→ 过拟合信号；
  - score 均值 <95 或单 run <90 → 过拟合信号；
  - 新热点树（42/43/45 未见的 tree#）出现只记档案不算杀。

用法:
  python3 tools/genval_run.py smoke           # Phase 0：S46_r0 证据门
  python3 tools/genval_run.py rest <outdir>   # Phase 1：其余 cells 续跑
  python3 tools/genval_run.py all             # 全 10 cells（跳过 Phase 0 门）
"""
import glob
import os
import re
import shutil
import subprocess
import sys
import time

import yaml

WS = os.path.expanduser("~/zx2026_arena_ws")


def _time_limit_s():
    try:
        with open(WS + "/src/zx2026_common/config/competition_rules.yaml") as f:
            return float(yaml.safe_load(f).get("time_limit_s", 600.0))
    except Exception:
        return 600.0


sys.path.insert(0, os.path.join(WS, "tools"))
from matrix_run import CFG, VERIFY_TXT, SMOKE_LOG, set_in_block, \
    set_key_in_block, set_top, parse_metrics

ROS_LOG = os.path.expanduser("~/.ros/log")

SEEDS = (46, 47, 48, 49, 50)
REPEATS = (0, 1)
CELLS = [dict(tag="S%d_r%d" % (s, rep), seed=s)
         for rep in REPEATS for s in SEEDS]

EVIDENCE_PATS = [
    "closed_loop=",            # banner（含 soa=/swg=/swq= 回显）
    "_guard hit",              # 分离/群集 guard 触发
    "swarm quiet window",
    "collision marker at",     # 碰撞行（含 close=/clr= 遥测）
    "DEAD-END escape engaged",
    "dead-end escape done",
    "goal-seal",
]


def patch_cell(cell):
    """显式落盘全部旗（现默认栈全量；补丁跨 cell 累积，缺省也必须写）。"""
    set_top(CFG, "run_seed", cell["seed"])
    # ---- 现默认栈（W1 三旗 + W2-P2 + ② mem6，全 ON/OFF 显式） ----
    set_in_block(CFG, "temporal_consistency", "false")
    set_in_block(CFG, "wind", "false")
    set_in_block(CFG, "comms", "false")
    set_in_block(CFG, "deadend_escape", "true")
    set_key_in_block(CFG, "deadend_escape", "exit_hyst_s", 2.5)
    set_in_block(CFG, "veto_gate", "false")
    set_in_block(CFG, "point_cache", "false")
    set_in_block(CFG, "drift_aware_margin", "false")
    set_key_in_block(CFG, "near_stop_zone", "enabled", "false")
    set_key_in_block(CFG, "nav", "inflation", 0.4)
    set_in_block(CFG, "rescue_mutex", "true")
    set_in_block(CFG, "bounce_corridor", "true")
    set_in_block(CFG, "sep_obs_guard", "true")
    set_in_block(CFG, "hotspot_inflate", "false")
    set_key_in_block(CFG, "via_slots", "enabled", "true")
    set_key_in_block(CFG, "rescue_quiet", "enabled", "true")
    set_key_in_block(CFG, "obs_guard", "enabled", "true")
    set_key_in_block(CFG, "vo_avoid", "enabled", "false")
    set_key_in_block(CFG, "boundary_margin", "enabled", "true")
    set_key_in_block(CFG, "return_route", "enabled", "false")
    set_key_in_block(CFG, "branch_handling", "contact_mark", "false")
    set_key_in_block(CFG, "branch_handling", "ca_iso_floor", 0.0)
    set_key_in_block(CFG, "branch_handling", "cloud_mem_frames", 6)


def capture_evidence(outdir):
    """从 ~/.ros/log 最新 run 目录采证据行 → <outdir>/evidence.txt。"""
    dirs = [d for d in glob.glob(os.path.join(ROS_LOG, "*"))
            if os.path.isdir(d) and os.path.basename(d) != "latest"]
    if not dirs:
        open(os.path.join(outdir, "evidence.txt"), "w").write("(no ros log dir)")
        return "(none)"
    run_dir = max(dirs, key=os.path.getmtime)
    lines = []
    for fp in sorted(glob.glob(os.path.join(run_dir, "*.log"))):
        try:
            txt = open(fp, encoding="utf-8", errors="replace").read()
        except OSError:
            continue
        base = os.path.basename(fp)
        for ln in txt.splitlines():
            if any(p in ln for p in EVIDENCE_PATS):
                lines.append("%s | %s" % (base, ln.strip()))
    with open(os.path.join(outdir, "evidence.txt"), "w", encoding="utf-8") as f:
        f.write("run_dir=%s\n" % os.path.basename(run_dir))
        f.write("\n".join(lines) + "\n")
    return os.path.basename(run_dir)


def _run_once(cell, outdir):
    """跑一次 run_verify.sh 并收集结果 + 证据；返回 (row, sleep_drift_s)。"""
    tag, seed = cell["tag"], cell["seed"]
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    patch_cell(cell)

    t0 = time.time()
    m0 = time.monotonic()
    timed_out = False
    try:
        subprocess.run(["bash", WS + "/run_verify.sh"],
                       capture_output=True, text=True,
                       timeout=_time_limit_s() + 300.0)
    except subprocess.TimeoutExpired:
        timed_out = True
        subprocess.run(["pkill", "-f", "rosmaster.*11411"], capture_output=True)
        subprocess.run(["pkill", "-f", "zx2026_all.launch"], capture_output=True)
        time.sleep(2)
    # 封卷汇合窗：report_settle_s 延迟写可能落在 run_verify 退出之后（W2 race）。
    time.sleep(3.0)
    dt = time.time() - t0
    drift = dt - (time.monotonic() - m0)

    row = dict(tag=tag, seed=seed, verdict="FAIL", score=-1, done_t=-1,
               col=-1, stuck_s=-1, stuck_n=-1, flips=-1, dist=-1, minclr=-1,
               runtime_s=round(dt), timeout=timed_out, sleep_s=round(drift, 1))
    try:
        txt = open(VERIFY_TXT).read()
        if "VERDICT: PASS" in txt:
            row["verdict"] = "PASS"
        m = re.search(r"-- state -> DONE @ (\d+)s", txt)
        if m:
            row["done_t"] = int(m.group(1))
    except OSError:
        pass
    scores = glob.glob("/tmp/zx2026_score_*.yaml")
    if scores:
        try:
            sy = yaml.safe_load(open(max(scores, key=os.path.getmtime)))
            t = sy.get("total")
            if isinstance(t, (int, float)):
                row["score"] = int(t)
        except Exception:
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
    scores = glob.glob("/tmp/zx2026_score_*.yaml")
    if scores:
        shutil.copy(max(scores, key=os.path.getmtime),
                    os.path.join(outdir, "score.yaml"))
    run_dir = capture_evidence(outdir)
    row["run_dir"] = run_dir
    return row, drift


def run_cell(cell, outdir, rerun_on_sleep=True):
    """跑一个 cell；检出睡眠/挂死污染则留档并自动重跑一次。"""
    row, drift = _run_once(cell, outdir)
    contaminated = drift > 30.0 or (row["score"] == -1 and not row["timeout"])
    if contaminated and rerun_on_sleep:
        why = ("wall-monotonic 漂移 %.0fs" % drift) if drift > 30.0 \
            else "verify 无 total（rosmaster 残留挂死）"
        print("    !! %s —— cell 污染：留档 __contaminated，自动重跑" % why,
              flush=True)
        shutil.rmtree(outdir + "__contaminated", ignore_errors=True)
        try:
            os.rename(outdir, outdir + "__contaminated")
        except OSError:
            pass
        row, drift = _run_once(cell, outdir)
    row["sleep_s"] = round(drift, 1)
    return row


def summary(rows):
    print("\n===== 泛化验证汇总（基线：B 臂近期 Σcol/run≈1.2、score 100） =====")
    for s in SEEDS:
        arm = [r for r in rows if r["seed"] == s]
        if not arm:
            continue
        pv = sum(1 for r in arm if r["verdict"] == "PASS")
        col = [r.get("col", -1) for r in arm]
        sc_ = [r.get("score", -1) for r in arm]
        dn = [r.get("done_t", -1) for r in arm]
        mc = [r.get("minclr", -1) for r in arm if r.get("minclr", -1) >= 0]
        print("seed%-3d PASS %d/%d  col=%s  score=%s  done_t=%s  minclr=%s" % (
            s, pv, len(arm), col, sc_, dn,
            ["%.3f" % m for m in mc] if mc else "-"))
    print("毙杀：FAIL / Σcol均值超基线 / score均值<95；新热点树只记档案。")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    outbase = WS + "/run_logs/genval_%s" % time.strftime("%Y%m%d_%H%M%S")
    if mode == "smoke":
        cell = CELLS[0]
        outdir = outbase + "_%s" % cell["tag"]
        print("Phase 0 smoke: %s -> %s" % (cell["tag"], outdir), flush=True)
        row = run_cell(cell, outdir)
        print("smoke row:", row, flush=True)
        if row["verdict"] != "PASS" or row["score"] < 0:
            print("SMOKE FAIL — 不进入矩阵", flush=True)
            sys.exit(2)
        print("SMOKE PASS", flush=True)
        return
    if mode == "rest":
        outbase = sys.argv[2]
        rows = []
        # 按 tag 存在性去重：cell 目录已存在=已完成跳过；__contaminated 改名后
        # 原 tag 目录不在=自动重跑。
        cells = [c for c in CELLS
                 if not glob.glob(outbase + "_" + c["tag"])]
    else:
        cells = CELLS
        rows = []
    for i, cell in enumerate(cells):
        outdir = "%s_%s" % (outbase, cell["tag"])
        print("[%d/%d] %s ..." % (i + 1, len(cells), cell["tag"]), flush=True)
        row = run_cell(cell, outdir)
        print("    ", row, flush=True)
        rows.append(row)
        summary(rows)
    print("DONE -> %s" % outbase, flush=True)


if __name__ == "__main__":
    main()
