#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w1_matrix_run.py — W1 swarm 层治理/via_slots A/B 矩阵（2026-09-03）。

基线 = 当前默认栈显式落盘（滚动基线纪律）：de+hyst2.5 / rescue_mutex /
bounce_corridor / sep_obs_guard 全开，hi/veto/pc/dam/nstop/tc/wind/comms
全关，inflation 0.4。臂（每 seed 交错 A→D，防时段漂移混淆臂间差异）：
  A_base    W1 三旗全关
  B_vs      via_slots
  C_vs_rq   via_slots + rescue_quiet
  D_full    via_slots + rescue_quiet + swarm_obs_guard
seeds: 42,43,45（真值森林 density_seed=42 固定，forest 不随 run_seed 变）。
cell 证据（banner/guard hits/quiet-window/VIA-SLOT/碰撞行）从 ~/.ros/log
最新 run 目录采入 <cell>/evidence.txt（节点日志不在 smoke.log——roslaunch
重定向到 per-node 文件，collision_extract.sh 同源）。

复用 matrix_run.py 的补丁机器与指标解析；不直接改跑其 _run_once（它把
sep_obs_guard 强制回 false——写于翻默认前，已陈旧，勿复用）。

用法:
  python3 tools/w1_matrix_run.py smoke           # Phase 0：A_base seed42 + 证据断言
  python3 tools/w1_matrix_run.py rest <outdir>   # Phase 1：其余 11 cells 续跑同一矩阵
  python3 tools/w1_matrix_run.py all             # 全 12 cells（跳过 Phase 0 门）
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
sys.path.insert(0, os.path.join(WS, "tools"))
from matrix_run import CFG, VERIFY_TXT, SMOKE_LOG, set_in_block, \
    set_key_in_block, set_top, parse_metrics

ROS_LOG = os.path.expanduser("~/.ros/log")

CELL_FLAGS = [
    ("A_base", dict(vs=False, rq=False, swg=False)),
    ("B_vs", dict(vs=True, rq=False, swg=False)),
    ("C_vs_rq", dict(vs=True, rq=True, swg=False)),
    ("D_full", dict(vs=True, rq=True, swg=True)),
]
SEEDS = (42, 43, 45)
CELLS = [dict(tag=t, seed=s, **f) for s in SEEDS for (t, f) in CELL_FLAGS]

EVIDENCE_PATS = [
    "closed_loop=",            # banner（含 soa=/swg=/swq= 回显）
    "_guard hit",              # 分离/群集 guard 触发（分桶遥测）
    "swarm quiet window",      # rescue_quiet 爬坡窗开启
    "VIA-SLOT",                # via_slots 槽位选定
    "via_slots",               # 回退/告警行
    "heading to via slot",
    "heading to crossing zone",
    "collision marker at",     # 碰撞行（含 close=/clr= 遥测）
    "DEAD-END escape engaged",
    "dead-end escape done",
    "goal-seal",
]


def patch_cell(cell):
    """显式落盘全部旗（基线栈 + W1）：补丁跨 cell 累积，缺省也必须写。"""
    set_top(CFG, "run_seed", cell["seed"])
    # ---- 基线栈（滚动基线 = 上一波默认栈） ----
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
    # ---- W1 三旗 ----
    set_key_in_block(CFG, "via_slots", "enabled",
                     "true" if cell.get("vs") else "false")
    set_key_in_block(CFG, "rescue_quiet", "enabled",
                     "true" if cell.get("rq") else "false")
    set_key_in_block(CFG, "obs_guard", "enabled",
                     "true" if cell.get("swg") else "false")


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
    # 清理上一 cell 可能残留的节点（防 collector 双实例交错写 CSV）
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    patch_cell(cell)

    t0 = time.time()
    m0 = time.monotonic()
    timed_out = False
    try:
        subprocess.run(["bash", WS + "/run_verify.sh"],
                       capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        timed_out = True
        subprocess.run(["pkill", "-f", "rosmaster.*11411"], capture_output=True)
        subprocess.run(["pkill", "-f", "zx2026_all.launch"], capture_output=True)
        time.sleep(2)
    dt = time.time() - t0
    drift = dt - (time.monotonic() - m0)

    row = dict(tag=tag, seed=seed, verdict="FAIL", score=-1, done_t=-1,
               col=-1, stuck_s=-1, stuck_n=-1, flips=-1, dist=-1, minclr=-1,
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
    # 挂死污染签名：verify 无 total（rosmaster 残留 → 起飞空转 ~456s，指标
    # 逐字节继承上一 cell）。非 timeout 的 score=-1 一律按污染重跑一次。
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


COLS = ["tag", "seed", "verdict", "score", "done_t", "col", "stuck_s",
        "stuck_n", "flips", "dist", "minclr", "runtime_s", "sleep_s",
        "timeout"]


def append_row(csv_path, row, write_header):
    with open(csv_path, "a") as f:
        if write_header:
            f.write(",".join(COLS) + "\n")
        f.write(",".join(str(row[c]) for c in COLS) + "\n")


def summarize(rows):
    print("\n===== SUMMARY (per arm) =====", flush=True)
    for tag in sorted({r["tag"] for r in rows}):
        arm = [r for r in rows if r["tag"] == tag]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / max(n, 1)
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%-8s n=%d pass=%d score=%.1f done=%.0fs col=%.1f stuck=%.1fs "
              "minclr=%.3f" % (tag, n, passes, mean("score"), mean("done_t"),
                               mean("col"), mean("stuck_s"), mean("minclr")),
              flush=True)
    print("\n===== 同 seed 配对 =====", flush=True)
    for s in SEEDS:
        seg = [r for r in rows if r["seed"] == s]
        print("seed %d: %s" % (s, " | ".join(
            "%s col=%s score=%s %s" % (r["tag"], r["col"], r["score"],
                                       r["verdict"]) for r in seg)), flush=True)


def smoke_gate(outdir):
    """Phase 0 证据断言：banner 在 + W1 旗关回显 + VIA-SLOT 缺席 + close= 在。"""
    ev = open(os.path.join(outdir, "00_A_base_seed42", "evidence.txt")).read()
    ok, msgs = True, []
    m = re.search(r"soa=(\w+) swg=(\w+) swq=(\w+)", ev)
    if not m:
        ok = False
        msgs.append("FAIL: banner (soa=/swg=/swq=) not found")
    else:
        msgs.append("banner: %s" % m.group(0))
        if not (m.group(1) == "True" and m.group(2) == "False"
                and m.group(3) == "False"):
            ok = False
            msgs.append("FAIL: banner flags wrong (期望 soa=True swg=False swq=False)")
    if "VIA-SLOT" in ev:
        ok = False
        msgs.append("FAIL: VIA-SLOT present with flag off")
    else:
        msgs.append("VIA-SLOT absent (flag off) OK")
    n_col = len(re.findall(r"collision marker at", ev))
    if n_col:
        if "close=" not in ev:
            ok = False
            msgs.append("FAIL: collision lines lack close= telemetry")
        else:
            msgs.append("collision lines carry close= telemetry (%d events)" % n_col)
    else:
        msgs.append("zero collisions in smoke (close= check deferred to matrix)")
    n_grd = len(re.findall(r"_guard hit", ev))
    msgs.append("sep_guard hits in smoke: %d (sep_obs_guard 默认开，纯遥测新行)" % n_grd)
    return ok, msgs


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    if mode == "smoke":
        ts = time.strftime("%Y%m%d_%H%M%S")
        outroot = os.path.join(WS, "run_logs", "matrix_w1_" + ts)
        os.makedirs(outroot, exist_ok=True)
        backup = CFG + ".matrix_bak"
        shutil.copy(CFG, backup)
        try:
            print("[smoke] 00_A_base_seed42 running...", flush=True)
            row = run_cell(CELLS[0], os.path.join(outroot, "00_A_base_seed42"))
            append_row(os.path.join(outroot, "matrix_results.csv"), row, True)
            print("    -> %s score=%s col=%s minclr=%s"
                  % (row["verdict"], row["score"], row["col"], row["minclr"]),
                  flush=True)
        finally:
            shutil.copy(backup, CFG)
            os.remove(backup)
        ok, msgs = smoke_gate(outroot)
        print("\n===== PHASE 0 GATE =====", flush=True)
        for m in msgs:
            print("  " + m, flush=True)
        print("GATE: %s" % ("PASS" if ok else "FAIL"), flush=True)
        print("outdir: %s" % outroot, flush=True)
        if not ok:
            sys.exit(2)
        print("next: python3 tools/w1_matrix_run.py rest %s" % outroot,
              flush=True)
        return

    if mode == "rest":
        outroot = sys.argv[2]
        csv_path = os.path.join(outroot, "matrix_results.csv")
        done = set()
        if os.path.exists(csv_path):
            with open(csv_path) as f:
                for r in csv_rows(f):
                    done.add((r[0], int(float(r[1]))))
        todo = [c for c in CELLS if (c["tag"], c["seed"]) not in done]
        print("rest: %d cells todo in %s" % (len(todo), outroot), flush=True)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        outroot = os.path.join(WS, "run_logs", "matrix_w1_" + ts)
        todo = list(CELLS)

    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".matrix_bak"
    shutil.copy(CFG, backup)
    csv_path = os.path.join(outroot, "matrix_results.csv")
    new_rows = []
    try:
        for k, cell in enumerate(todo):
            name = "%02d_%s_seed%d" % (CELLS.index(cell), cell["tag"], cell["seed"])
            print("[%d/%d] %s running..." % (k + 1, len(todo), name), flush=True)
            row = run_cell(cell, os.path.join(outdir_cell(outroot, name)))
            append_row(csv_path, row, not os.path.exists(csv_path))
            new_rows.append(row)
            print("    -> %s score=%s done=%ss col=%s stuck=%.1fs minclr=%s (%.0fs)"
                  % (row["verdict"], row["score"], row["done_t"], row["col"],
                     row["stuck_s"], row["minclr"], row["runtime_s"]), flush=True)
    finally:
        shutil.copy(backup, CFG)
        os.remove(backup)
    # 汇总读全量 CSV（含 smoke 行）
    all_rows = []
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            for r in csv_rows(f):
                all_rows.append(dict(zip(COLS, r[:len(COLS)])))
        for r in all_rows:
            for k in ("score", "done_t", "col", "stuck_s", "stuck_n", "flips",
                      "dist", "minclr"):
                r[k] = float(r[k])
    summarize(all_rows)
    print("results csv: %s" % csv_path, flush=True)


def outdir_cell(outroot, name):
    p = os.path.join(outroot, name)
    os.makedirs(p, exist_ok=True)
    return p


def csv_rows(f):
    import csv as _csv
    for r in _csv.reader(f):
        if len(r) >= len(COLS) and r[0] != "tag":
            yield r


if __name__ == "__main__":
    main()
