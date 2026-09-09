#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/matrix_post.py — 矩阵后处理：修复 FAIL 单元格的指标（P0 产出消费方）。

FAIL run 没有到达 DONE，collector 不写时间戳定稿 YAML → matrix_run 的 glob 会
拿到上一个 run 的旧文件（陈旧指标）。本脚本：
  1. 对每个 cell，若归档 nav_metrics.yaml 的 state != DONE，则从 CSV 恢复：
     smoke.log 的 ROS 时间戳（sim 域）取 origin，取 CSV 中 run_seed 匹配且
     sim_t ∈ [origin, origin+700] 的行，每机取最后一行（计数器为累计值）聚合。
  2. 输出 matrix_results_fixed.csv + 每臂汇总表。

用法: python3 ~/zx2026_arena_ws/tools/matrix_post.py [matrix_dir]
"""
import csv
import glob
import os
import re
import sys

import yaml

WS = os.path.expanduser("~/zx2026_arena_ws")


def _time_limit_s():
    """P6：时限从 competition_rules.yaml 读取（回填窗=时限+60）。"""
    try:
        with open(WS + "/src/zx2026_common/config/competition_rules.yaml") as f:
            return float(yaml.safe_load(f).get("time_limit_s", 600.0))
    except Exception:
        return 600.0


WINDOW_S = _time_limit_s() + 60.0
CSV_PATH = os.path.join(WS, "run_logs", "nav_metrics_ts.csv")
COLS = ["tag", "seed", "verdict", "score", "done_t", "col", "stuck_s",
        "stuck_n", "flips", "dist", "minclr", "runtime_s", "timeout"]

FIELDS = ["collisions", "stuck_s", "stuck_n", "stuck_max_s", "flips",
          "dist_m", "min_clear_m", "max_speed", "sim_t", "speed", "clear"]


def load_csv_rows():
    raw = open(CSV_PATH, "rb").read().replace(b"\x00", b"").decode("utf-8", "replace")
    rows = []
    for r in csv.reader(raw.splitlines()):
        if len(r) < 14:
            continue
        try:
            if r[0] != "run_seed":
                float(r[0])
        except ValueError:
            continue
        rows.append(r)
    return rows


def smoke_origin(smoke_path):
    """smoke.log 里最早的 ROS 时间戳（sim 域）。"""
    best = None
    pat = re.compile(r"\[(\d{10}\.\d+)\]:")
    try:
        with open(smoke_path, errors="replace") as f:
            for ln in f:
                m = pat.search(ln)
                if m:
                    t = float(m.group(1))
                    best = t if best is None else min(best, t)
    except OSError:
        return None
    return best


def agg_from_yaml(path):
    d = yaml.safe_load(open(path)) or {}
    drones = d.get("drones", {})
    state = d.get("state", "?")
    agg = {}
    for k, v in drones.items():
        agg[k] = {f: float(v.get(f, 0)) for f in FIELDS}
    return state, agg


def agg_from_csv(rows, seed, origin):
    """window 内每机取最后一行（计数器累计），聚合。"""
    last = {}
    for r in rows:
        try:
            if r[0] != str(seed):
                continue
            sim_t = float(r[11])
            if not (origin - 5 <= sim_t <= origin + WINDOW_S):
                continue
            last[r[2]] = r
        except (ValueError, IndexError):
            continue
    agg = {}
    for k, r in last.items():
        agg[k] = {f: float(v) for f, v in zip(FIELDS, r[3:14])}
    return agg


def summarize(agg):
    a = dict(col=0, stuck_s=0.0, stuck_n=0, flips=0, dist=0.0, minclr=1e9)
    for v in agg.values():
        a["col"] += int(v.get("collisions", 0))
        a["stuck_s"] += float(v.get("stuck_s", 0))
        a["stuck_n"] += int(v.get("stuck_n", 0))
        a["flips"] += int(v.get("flips", 0))
        a["dist"] += float(v.get("dist_m", 0))
        a["minclr"] = min(a["minclr"], float(v.get("min_clear_m", 1e9)))
    a["minclr"] = round(a["minclr"], 3) if a["minclr"] < 1e9 else -1
    a["stuck_s"] = round(a["stuck_s"], 1)
    a["dist"] = round(a["dist"], 1)
    return a


def main():
    mdir = sys.argv[1] if len(sys.argv) > 1 else sorted(
        glob.glob(os.path.join(WS, "run_logs", "matrix_*")))[-1]
    csv_in = os.path.join(mdir, "matrix_results.csv")
    out_path = os.path.join(mdir, "matrix_results_fixed.csv")
    all_rows = load_csv_rows()

    fixed = []
    with open(csv_in) as f:
        rd = csv.DictReader(f)
        for row in rd:
            cell_dir = None
            for d in glob.glob(os.path.join(mdir, "*_%s_seed%s" % (row["tag"], row["seed"]))):
                cell_dir = d
            yml = os.path.join(cell_dir, "nav_metrics.yaml") if cell_dir else None
            state, agg = ("?", {})
            if yml and os.path.exists(yml):
                state, agg = agg_from_yaml(yml)
            # 陈旧检测不能看 state：FAIL cell 归档到的正是上一个 PASS run 的
            # DONE yaml（state=DONE）。用矩阵 CSV 的 verdict 判定。
            if row["verdict"] != "PASS" and cell_dir:
                origin = smoke_origin(os.path.join(cell_dir, "smoke.log"))
                if origin is not None:
                    # 窗口上限 = verify 观测上限（时限+60s，P6 参数化；
                    # 同 seed 下一 cell 的 origin 至少在本 cell origin + runtime+过渡 之后）
                    agg = agg_from_csv(all_rows, int(row["seed"]), origin)
                    if agg:
                        print("recovered %s_seed%s from csv (origin=%.0f, %d drones)"
                              % (row["tag"], row["seed"], origin, len(agg)))
                    else:
                        print("WARN: %s_seed%s recovery empty, keeping stale"
                              % (row["tag"], row["seed"]))
            s = summarize(agg)
            row.update({"col": s["col"], "stuck_s": s["stuck_s"],
                        "stuck_n": s["stuck_n"], "flips": s["flips"],
                        "dist": s["dist"], "minclr": s["minclr"]})
            fixed.append(row)

    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLS)
        w.writeheader()
        for row in fixed:
            w.writerow(row)

    print("\n===== SUMMARY (mean per arm, fixed) =====")
    arms = {}
    for row in fixed:
        arms.setdefault(row["tag"], []).append(row)
    for tag in sorted(arms):
        arm = arms[tag]
        n = len(arm)
        mean = lambda k: sum(float(r[k]) for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%-8s n=%d pass=%d score=%.1f done=%.0fs col=%.1f stuck=%.1fs "
              "stuck_n=%.0f flips=%.0f dist=%.1f minclr=%.2f"
              % (tag, n, passes, mean("score"), mean("done_t"), mean("col"),
                 mean("stuck_s"), mean("stuck_n"), mean("flips"),
                 mean("dist"), mean("minclr")))
    print("fixed csv:", out_path)


if __name__ == "__main__":
    main()
