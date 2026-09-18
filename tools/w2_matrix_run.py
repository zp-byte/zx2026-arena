#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w2_matrix_run.py — W2 走廊 A/B 矩阵（2026-09-17，docs/wave2_corridor_case.md）。

基线 = 当前默认栈显式落盘（滚动基线纪律，w1_matrix_run 同款机器）：de+hyst2.5 /
rescue_mutex / bounce_corridor / sep_obs_guard / via_slots+zone_margin1.0 /
obs_guard / rescue_quiet 全开，inflation 0.4，tc/wind/comms/veto/pc/dam/
near_stop/hi/vo 全关。臂（每 seed 交错 A→D，防时段漂移混淆）：
  A_base    当前默认栈（W1 三旗 ON，W2 双旗 OFF）
  B_p2      + boundary_margin（fence 反应层 z-aware 纳入）
  C_p1      + return_route（中缝 via_y 8.5，离线核查 w2_return_route_check.py）
  D_p1p2    双开（单独生效都可能改变走廊形态，合开臂必须测——立案执行序 2）
seeds: 42,43,45（立案口径；真值森林 density_seed=42 固定）。

毙杀口径（立案预登记）：
  P1：返航时长 +20s 以上、或南绕致 DROP_SIDE/zone 碰撞上升 → 回退；
      签名 RETURN y>11 采样=0 / NE+curb 碰撞=0 / path_length <15%。
  P2：走廊/返航超窗或新增 FAIL → 调参回退；签名 fence/curb 事件→0。
  通用：done/score 不得劣化（B 臂纪律同 gz_closure）。

用法:
  python3 tools/w2_matrix_run.py smoke           # Phase 0：A_base seed42 证据门
  python3 tools/w2_matrix_run.py rest <outdir>   # Phase 1：其余 11 cells 续跑
  python3 tools/w2_matrix_run.py all             # 全 12 cells（跳过 Phase 0 门）
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
    ("A_base", dict(bm=False, rr=False)),
    ("B_p2", dict(bm=True, rr=False)),
    ("C_p1", dict(bm=False, rr=True)),
    ("D_p1p2", dict(bm=True, rr=True)),
]
SEEDS = (42, 43, 45)
CELLS = [dict(tag=t, seed=s, **f) for s in SEEDS for (t, f) in CELL_FLAGS]

EVIDENCE_PATS = [
    "closed_loop=",            # banner（含 soa=/swg=/swq= 回显）
    "_guard hit",              # 分离/群集 guard 触发
    "swarm quiet window",
    "VIA-SLOT",
    "heading to via slot",
    "RETURN-VIA",              # W2-P1 中缝路点选定/到达
    "return start",            # 返航腿起点（返航时长证据锚）
    "TOUCHDOWN",               # 返航终点（返航时长证据锚）
    "collision marker at",     # 碰撞行（含 close=/clr= 遥测）
    "DEAD-END escape engaged",
    "dead-end escape done",
    "goal-seal",
]


def patch_cell(cell):
    """显式落盘全部旗（基线栈 + W2）：补丁跨 cell 累积，缺省也必须写。"""
    set_top(CFG, "run_seed", cell["seed"])
    # ---- 基线栈（滚动基线 = 当前默认栈；W1 三旗 matrix_w1b 后已翻默认） ----
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
    # W1 三旗（新基线=全开，勿复用 W1 矩阵的 A=全关形态——立案执行序 4 注记）
    set_key_in_block(CFG, "via_slots", "enabled", "true")
    set_key_in_block(CFG, "rescue_quiet", "enabled", "true")
    set_key_in_block(CFG, "obs_guard", "enabled", "true")
    # W6 VO-lite（默认关维持）
    set_key_in_block(CFG, "vo_avoid", "enabled", "false")
    # ---- W2 双旗 ----
    set_key_in_block(CFG, "boundary_margin", "enabled",
                     "true" if cell.get("bm") else "false")
    set_key_in_block(CFG, "return_route", "enabled",
                     "true" if cell.get("rr") else "false")


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
    # 退出之后——不等待就 parse/copy 会拿到未定稿报告（D_s43 行 PASS 文件 FAIL
    # 的 race 真身：预写块 PASS、定稿块 FAIL）。等过结算窗再读。
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
    # score：从裁判报告 yaml 读 total（verify.txt 的 score_summary 现为逐机
    # 行、无 total= 汇总行——w1 时代正则陈旧，A42 smoke 误判污染的真身）
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
    print("\n===== W2 矩阵汇总 =====")
    for t, _ in CELL_FLAGS:
        arm = [r for r in rows if r["tag"] == t]
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
    print("毙杀口径：P1 返航 +20s/南绕碰撞上升；P2 超窗/新增 FAIL；")
    print("        通用 done/score 不劣化。返航时长从 evidence.txt 的")
    print("        return start/TOUCHDOWN 行对账。")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    outbase = WS + "/run_logs/w2_matrix_%s" % time.strftime("%Y%m%d_%H%M%S")
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
            m = re.match(r".*_([A-Za-z_]+)_s(\d+)$", d)
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
