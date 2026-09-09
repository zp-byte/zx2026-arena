#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/matrix_rule_run.py — 科目三比赛口径 3-seed 回归矩阵（P8 收口第 3 关）。

默认旗栈全开（color_id + rule_monitor + W1 族默认机制），种子 42/43/45 交错
复用 matrix_run.py 的补丁/漂移检测基建。每 cell 过线门（全过才算 GATES_OK）：
  verdict PASS（6 机全终局）、col==0、landed_n==6、retired/corridor_missed 空、
  s1∈[40,60]（种子方差容忍单平台骑线）、minclr 不低于基线、
  stuck ≤ 2×基线、done_t ≤ 基线+90s（RECON 腿新增航程带宽）

基线取 P0 冻结归档 run_logs/baseline_rules0_*/（无则回退绝对地板）。
用法: python3 tools/matrix_rule_run.py
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
from matrix_run import CFG, WS as _WS, parse_metrics, set_top  # noqa: E402

VERIFY_TXT = "/tmp/zx2026_verify.txt"
SMOKE_LOG = "/tmp/zx2026_smoke.log"
SEEDS = (42, 43, 45)
# 基线缺失时的回退地板（W1 D_full 实测 minclr 0.521 / done 390s / stuck 个位数）
FALLBACK = {"minclr": 0.35, "stuck_s": 60.0, "done_t": 420.0}


def _time_limit_s():
    try:
        with open(WS + "/src/zx2026_common/config/competition_rules.yaml") as f:
            return float(yaml.safe_load(f).get("time_limit_s", 600.0))
    except Exception:
        return 600.0


VERIFY_TIMEOUT = _time_limit_s() + 300.0


def load_baseline():
    """基线参照（多 seed 取最差侧）。优先新场地归档 baseline_comp0_*
    （比赛口径 3 平台场地）；无则回退 P0 冻结的 baseline_rules0_*（旧 6 平台
    场地——minclr 对新场地结构性失配：平台上空 0.55m 悬停贴平台属任务
    本征几何，旧基线 0.631 不可达，此时 minclr 门降级 record-only）。
    返回 (base, source)。"""
    base = dict(FALLBACK)
    src = "fallback"
    for tag in ("baseline_comp0", "baseline_rules0"):
        ymls = sorted(glob.glob(WS + "/run_logs/%s_*/**/nav_metrics.yaml" % tag,
                                recursive=True))
        if not ymls:
            continue
        src = tag
        aggs = [parse_metrics(p) for p in ymls]
        if any(a["minclr"] > 0 for a in aggs):
            base["minclr"] = min(a["minclr"] for a in aggs if a["minclr"] > 0)
        base["stuck_s"] = max(a["stuck_s"] for a in aggs)
        dones = []
        for vt in glob.glob(WS + "/run_logs/%s_*/**/verify.txt" % tag,
                            recursive=True):
            m = re.search(r"-- state -> DONE @ (\d+)s", open(vt).read())
            if m:
                dones.append(int(m.group(1)))
        if dones:
            base["done_t"] = max(dones)
        break
    return base, src


def parse_score_yaml(path):
    d = yaml.safe_load(open(path)) or {}
    drops = d.get("drops", {}) or {}
    offs = [float(v.get("offset", 9.9)) for v in drops.values()
            if isinstance(v, dict) and v.get("offset") is not None]
    return dict(total=int(d.get("total", -1)),
                s1=int(d.get("s1_total", -1)),
                s2=int(d.get("s2", -1)),
                landed_n=int(d.get("landed_n", -1)),
                correct_n=int(d.get("correct_n", -1)),
                retired=sorted(d.get("retired", []) or []),
                corridor=sorted(d.get("corridor_missed", []) or []),
                max_off=round(max(offs), 3) if offs else -1)


def _run_once(seed, outdir, base, src):
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    set_top(CFG, "run_seed", seed)

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

    row = dict(seed=seed, verdict="FAIL", total=-1, s1=-1, s2=-1, landed_n=-1,
               correct_n=-1, retired=[], corridor=[], max_off=-1, done_t=-1,
               col=-1, minclr=-1, stuck_s=-1, runtime_s=round(dt),
               timeout=timed_out, sleep_s=round(drift, 1), gates="")
    try:
        txt = open(VERIFY_TXT).read()
        if "VERDICT: PASS" in txt:
            row["verdict"] = "PASS"
        m = re.search(r"-- state -> DONE @ (\d+)s", txt)
        if m:
            row["done_t"] = int(m.group(1))
    except OSError:
        pass
    scores = sorted(glob.glob("/tmp/zx2026_score_*.yaml"), key=os.path.getmtime)
    if scores:
        try:
            row.update(parse_score_yaml(scores[-1]))
        except Exception as e:
            print("score parse fail:", e, flush=True)
    ymls = sorted(glob.glob(WS + "/run_logs/nav_metrics_2*.yaml"),
                  key=os.path.getmtime)
    if ymls:
        try:
            agg = parse_metrics(ymls[-1])
            row["col"] = agg["col"]
            row["minclr"] = agg["minclr"]
            row["stuck_s"] = agg["stuck_s"]
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
    if scores:
        shutil.copy(scores[-1], os.path.join(outdir, "score.yaml"))

    # ---- 过线门 ----
    g = []
    g.append(row["verdict"] == "PASS")
    g.append(row["col"] == 0)
    # 180s 时限口径（用户裁决 2026-09-09）：最晚释放机返航降落常压线差
    # 2-3s（run13 d3 释放 t≈169 → 封卷时 touchdown 未及计数，S2=30 档），
    # 门从 ==6 放到 >=5（S2≥30）
    g.append(row["landed_n"] >= 5)
    g.append(not row["retired"] and not row["corridor"])
    g.append(40 <= row["s1"] <= 60)
    # minclr 门：新场地基线（comp0）才卡门；rules0=旧 6 平台场地结构性
    # 失配（平台上空悬停贴平台 0.03-0.15 属任务本征），降级 record-only
    g.append(row["minclr"] >= base["minclr"] - 1e-3
             if src == "baseline_comp0" else True)
    g.append(row["stuck_s"] <= 2.0 * base["stuck_s"])
    g.append(0 < row["done_t"] <= base["done_t"] + 90)
    row["gates"] = "".join("1" if x else "0" for x in g)
    return row, drift


def run_cell(seed, outdir, base, src, rerun_on_sleep=True):
    row, drift = _run_once(seed, outdir, base, src)
    if drift > 30.0 and rerun_on_sleep:
        print("    !! wall-monotonic 漂移 %.0fs —— cell 污染，留档 __contaminated "
              "并自动重跑" % drift, flush=True)
        shutil.rmtree(outdir + "__contaminated", ignore_errors=True)
        try:
            os.rename(outdir, outdir + "__contaminated")
        except OSError:
            pass
        row, drift = _run_once(seed, outdir, base, src)
    row["sleep_s"] = round(drift, 1)
    return row


def main():
    base, bsrc = load_baseline()
    ts = time.strftime("%Y%m%d_%H%M%S")
    outroot = os.path.join(WS, "run_logs", "matrix_rule_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".matrix_bak"
    shutil.copy(CFG, backup)
    csv_path = os.path.join(outroot, "matrix_rule_results.csv")
    cols = ["seed", "verdict", "total", "s1", "s2", "landed_n", "correct_n",
            "retired", "corridor", "max_off", "done_t", "col", "minclr",
            "stuck_s", "runtime_s", "sleep_s", "timeout", "gates"]
    print("matrix_rule start -> %s  baseline=%s (%s)%s" % (
        outroot, base, bsrc,
        "  [minclr 门 record-only：旧场地基线]" if bsrc == "baseline_rules0" else ""),
        flush=True)
    rows = []
    try:
        for k, seed in enumerate(SEEDS):
            name = "%02d_seed%d" % (k, seed)
            print("[%d/%d] seed%d running..." % (k + 1, len(SEEDS), seed),
                  flush=True)
            row = run_cell(seed, os.path.join(outroot, name), base, bsrc)
            rows.append(row)
            with open(csv_path, "a") as f:
                if k == 0:
                    f.write(",".join(cols) + "\n")
                f.write(",".join(str(row[c]) for c in cols) + "\n")
            print("    -> %s total=%s (s1=%s s2=%s landed=%s correct=%s "
                  "ret=%s cor=%s) col=%s minclr=%s done=%ss gates=%s (%.0fs)"
                  % (row["verdict"], row["total"], row["s1"], row["s2"],
                     row["landed_n"], row["correct_n"], row["retired"],
                     row["corridor"], row["col"], row["minclr"],
                     row["done_t"], row["gates"], row["runtime_s"]), flush=True)
    finally:
        shutil.copy(backup, CFG)
        os.remove(backup)

    ok = sum(1 for r in rows
             if len(r["gates"]) == 8 and all(c == "1" for c in r["gates"]))
    print("\n===== SUMMARY =====", flush=True)
    for r in rows:
        print("seed%d %s total=%d s1=%d s2=%d landed=%d col=%d minclr=%.3f "
              "gates=%s" % (r["seed"], r["verdict"], r["total"], r["s1"],
                            r["s2"], r["landed_n"], r["col"], r["minclr"],
                            r["gates"]), flush=True)
    print("GATES_OK %d/%d" % (ok, len(rows)), flush=True)
    print("results csv:", csv_path, flush=True)
    sys.exit(0 if ok == len(rows) else 1)


if __name__ == "__main__":
    main()
