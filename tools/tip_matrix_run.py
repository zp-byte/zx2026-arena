#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/tip_matrix_run.py — ② 枝梢端点缺口 A/B（2026-09-18，cdd6075 法证续）。

立案（cdd6075）：H 臂残余 0.7 col = branch_tip t=1.00 端点擦碰 1-5mm、I 臂同梢
1cm 复现——云记忆 0.3s 过期 vs 1.8m/s 贴脸时窗是残余机制缺口。定量：react_r=
0.35+0.15+ca_react_extra 0.6=1.10m，1.8m/s 穿越反应窗需 0.41s > 记忆窗 3 帧
0.3s——梢点在窗入口Seen 一次，0.3s 后遗忘时还剩 ~0.19m 贴脸。

臂（2 旗 × seed{42,43,45} × 2 遍 = 12 cells，rep 交错 A→B 防时段漂移）：
  A_mem3    现默认 cloud_mem_frames=3（滚动基线：W1 三旗 + W2-P2 boundary_margin ON）
  B_mem6    cloud_mem_frames=6（0.6s=1.08m ≥ 0.41s 穿窗 + 盲帧余量）
伴生修复（两臂共同、先行落地）：孤立点阈值 ≤3 → ≤max(3, frames)——延窗后
梢点 6 帧回声不再被 ③ 误判密簇（nav_node 20260918）。

毙杀口径（预登记）：
  B 翻默认：Σcol 降（或 branch_tip 起数降且 Σcol 不升）∧ score 无 seed 劣化
            ∧ 零 FAIL ∧ done_t 方差带内。
  B 判毙：任一 FAIL / score 降 / 碰撞挪移到别的障碍类（J_hiso 风险再分布
          教训——归因对账）/ done_t 回归 >20s。
  minclr 仅参考：记忆加长让身后点驻留 0.6s，读数偏保守（指标敏感非行为敏感）。

用法:
  python3 tools/tip_matrix_run.py smoke           # Phase 0：A_mem3 seed42 证据门
  python3 tools/tip_matrix_run.py rest <outdir>   # Phase 1：其余 cells 续跑
  python3 tools/tip_matrix_run.py all             # 全 12 cells（跳过 Phase 0 门）
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

CELL_FLAGS = [
    ("A_mem3", dict(mem=3)),
    ("B_mem6", dict(mem=6)),
]
SEEDS = (42, 43, 45)
REPEATS = (0, 1)
CELLS = [dict(tag="%s_r%d" % (t, rep), seed=s, **f)
         for rep in REPEATS for s in SEEDS for (t, f) in CELL_FLAGS]

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
    """显式落盘全部旗（基线栈 + 枝梢时窗）：补丁跨 cell 累积，缺省也必须写。"""
    set_top(CFG, "run_seed", cell["seed"])
    # ---- 基线栈（滚动基线 = W1 三旗 + W2-P2 boundary_margin 全 ON） ----
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
    # ---- WaveB ①③ pilot 旗钉住（默认关），② 本矩阵自变量 ----
    set_key_in_block(CFG, "branch_handling", "contact_mark", "false")
    set_key_in_block(CFG, "branch_handling", "ca_iso_floor", 0.0)
    set_key_in_block(CFG, "branch_handling", "cloud_mem_frames", cell["mem"])


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
    # 封卷汇合窗：scorekeeper 的 report_settle_s=1.0 延迟写可能落在 run_verify
    # 退出之后——不等待就 parse/copy 会拿到未定稿报告（W2 D_s43 race 真身）。
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
    # score：从裁判报告 yaml 读 total（verify.txt 的 score_summary 无 total 行）。
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
    print("\n===== 枝梢时窗矩阵汇总 =====")
    for t, _ in CELL_FLAGS:
        arm = [r for r in rows if r["tag"].startswith(t + "_")]
        if not arm:
            continue
        pv = sum(1 for r in arm if r["verdict"] == "PASS")
        col = [r.get("col", -1) for r in arm]
        sc_ = [r.get("score", -1) for r in arm]
        dn = [r.get("done_t", -1) for r in arm]
        mc = [r.get("minclr", -1) for r in arm if r.get("minclr", -1) >= 0]
        print("%-8s PASS %d/%d  col=%s  score=%s  done_t=%s  minclr=%s" % (
            t, pv, len(arm), col, sc_, dn,
            ["%.3f" % m for m in mc] if mc else "-"))
    print("毙杀口径：B 翻默认=Σcol 降+score 平+零 FAIL+done_t 平；")
    print("        判毙=FAIL/score 降/碰撞挪移（归因对账）/done_t 超 20s。")
    print("        minclr 仅参考（记忆加长读数偏保守）。")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    outbase = WS + "/run_logs/tip_matrix_%s" % time.strftime("%Y%m%d_%H%M%S")
    if mode == "smoke":
        cell = CELLS[0]
        outdir = outbase + "_%s_s%d" % (cell["tag"], cell["seed"])
        print("Phase 0 smoke: %s seed%d -> %s" % (cell["tag"], cell["seed"],
                                                  outdir), flush=True)
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
        done = set()
        for d in glob.glob(outbase + "_*"):
            m = re.match(r".*_([A-Za-z0-9_]+_r\d+)_s(\d+)$", d)
            if m and not d.endswith("__contaminated"):
                done.add((m.group(1), int(m.group(2))))
        cells = [c for c in CELLS if (c["tag"], c["seed"]) not in done]
    else:
        cells = CELLS
        rows = []
    for i, cell in enumerate(cells):
        outdir = "%s_%s_s%d" % (outbase, cell["tag"], cell["seed"])
        print("[%d/%d] %s seed%d ..." % (i + 1, len(cells), cell["tag"],
                                         cell["seed"]), flush=True)
        row = run_cell(cell, outdir)
        print("    ", row, flush=True)
        rows.append(row)
        summary(rows)
    print("DONE -> %s" % outbase, flush=True)


if __name__ == "__main__":
    main()
