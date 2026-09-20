#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w7_matrix_run.py — W7 冠撞修复 A/B 矩阵（2026-09-20 冠撞法证续）。

立案（memory/zx2026-gz-crown-collision-forensics）：2026-09-20 gz 实弹 4 撞
全落枝冠高度零树干——①弹向级联+逃逸盲推 ②TIP 超模型 ③冠缘欠采样。
本矩阵验证 C1/C2（python 后端 substrate，机制后端无关）；C3 tip_slow 留
pilot 串行不在本矩阵。

臂（3 臂 × seed{42,43,45} = 9 cells，seed 内 A→B1→B2 交错防时段漂移）：
  A    滚动基线（W1 三旗+W2-P2+mem6 全 ON；W7 三旗全 OFF=零侵入回归）
  B1   +post_hit_calm（C1 撞后镇定窗：hold_s=3 逃逸冻结+vcap 0.8×2s）
  B2   +C1+C2 hazard_share（热点舰队广播 /zx2026/hazard_cells）

毙杀口径（预登记）：
  B1/B2 翻线：Σcol 降 ∧ score 无 seed 劣化 ∧ 零 FAIL ∧ done_t 方差带内
             ∧ slack 不劣化（W4 预算律 41.3s）。
  判毙：任一 FAIL / score 降 / 碰撞挪移到别的障碍类（归因对账）/
        done_t 回归 >20s。
  证据门：B1 臂每撞须见 POST-HIT CALM 行（FREEZE/HOLD 至少其一）；
        B2 臂须见 HAZARD-SHARE 行——无触发行=机制未咬合，col 差异不可归因。

用法:
  python3 tools/w7_matrix_run.py smoke           # Phase 0：A seed42 零侵入门
  python3 tools/w7_matrix_run.py rest <outdir>   # 其余 cells 续跑
  python3 tools/w7_matrix_run.py all             # 全 9 cells（跳过 Phase 0 门）
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
from matrix_run import CFG, VERIFY_TXT, SMOKE_LOG, set_top, set_in_block, \
    set_key_in_block, parse_metrics

ROS_LOG = os.path.expanduser("~/.ros/log")

CELL_FLAGS = [
    ("A", dict(ph=False, hz=False)),
    ("B1", dict(ph=True, hz=False)),
    ("B2", dict(ph=True, hz=True)),
]
SEEDS = (42, 43, 45)
CELLS = [dict(tag=t, seed=s, **f) for s in SEEDS for (t, f) in CELL_FLAGS]

EVIDENCE_PATS = [
    "closed_loop=",            # banner（含 ph=/hz=/tip= 回显）
    "_guard hit",              # 分离/群集 guard 触发
    "swarm quiet window",
    "collision marker at",     # 碰撞行（含 close=/clr= 遥测）
    "DEAD-END escape engaged",
    "dead-end escape done",
    "goal-seal",
    "POST-HIT CALM",           # W7-C1 触发证据（碰撞时刻）
    "POST-HIT FREEZE",         # W7-C1 冻结证据（活跃逃逸重选推迟）
    "POST-HIT HOLD",           # W7-C1 扣压证据（engagement 推迟）
    "HAZARD-SHARE",            # W7-C2 触发证据（收到他机危险格）
]


def patch_cell(cell):
    """显式落盘全部旗（滚动基线 + W7 臂变量）：补丁跨 cell 累积，缺省也必须写。"""
    set_top(CFG, "run_seed", cell["seed"])
    # ---- 滚动基线（= tip_matrix 20260918 同款：W1 三旗+W2-P2+mem6 全 ON） ----
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
    # ---- W7 臂变量（A 全 false=零侵入回归；B1=+C1；B2=+C1+C2） ----
    set_key_in_block(CFG, "post_hit_calm", "enabled",
                     "true" if cell["ph"] else "false")
    set_key_in_block(CFG, "hazard_share", "enabled",
                     "true" if cell["hz"] else "false")


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
    # W7 证据门计数：B 臂零触发行=机制未咬合（col 差异不可归因）
    ev = os.path.join(outdir, "evidence.txt")
    try:
        etxt = open(ev, encoding="utf-8").read()
    except OSError:
        etxt = ""
    for pat, key in (("POST-HIT CALM", "n_calm"), ("POST-HIT FREEZE", "n_frz"),
                     ("POST-HIT HOLD", "n_hold"), ("HAZARD-SHARE", "n_hz")):
        row[key] = etxt.count(pat)
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
    print("\n===== W7 冠撞三臂矩阵汇总 =====")
    for t, _ in CELL_FLAGS:
        arm = [r for r in rows if r["tag"] == t]
        if not arm:
            continue
        pv = sum(1 for r in arm if r["verdict"] == "PASS")
        col = [r.get("col", -1) for r in arm]
        sc_ = [r.get("score", -1) for r in arm]
        dn = [r.get("done_t", -1) for r in arm]
        mc = [r.get("minclr", -1) for r in arm if r.get("minclr", -1) >= 0]
        tr = [(r.get("n_calm", 0), r.get("n_hz", 0)) for r in arm]
        print("%-4s PASS %d/%d  col=%s  score=%s  done_t=%s  minclr=%s" % (
            t, pv, len(arm), col, sc_, dn,
            ["%.3f" % m for m in mc] if mc else "-"))
        print("     触发证据 (CALM,HAZARD)=%s" % (tr,))
    print("毙杀口径：翻线=Σcol 降+score 平+零 FAIL+done_t 平+slack 不劣化；")
    print("        判毙=FAIL/score 降/碰撞挪移（归因对账）/done_t 超 20s。")
    print("        证据门：B1 须见 POST-HIT *；B2 须见 HAZARD-SHARE（无=未咬合）。")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    outbase = WS + "/run_logs/w7_matrix_%s" % time.strftime("%Y%m%d_%H%M%S")
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
        # A 臂零侵入断言：W7 三旗关闭时零触发行
        if row.get("n_calm", 0) or row.get("n_hz", 0):
            print("SMOKE 零侵入 FAIL —— A 臂出现 W7 触发行（旗未关干净）",
                  flush=True)
            sys.exit(3)
        print("SMOKE PASS（零侵入门过：A 臂 W7 触发行=0）", flush=True)
        return
    if mode == "rest":
        outbase = sys.argv[2]
        rows = []
        done = set()
        for d in glob.glob(outbase + "_*"):
            m = re.match(r".*_([AB][12]?)_s(\d+)$", d)
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
