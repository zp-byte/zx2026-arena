#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/cam_matrix_run.py — 相机颜色检测 A/B 验证矩阵（P3，2026-09-10 相机立项）。

隔离变量只有两个：sim_settings.run_seed + competition_rules.color_id.source。
其余机制一律不动（用户默认档），避免把无关开关卷进 A/B 方差。

矩阵（交错执行，防时段漂移混淆臂间差异）：
  truth   color_id.source=truth   seeds 42-46   基线（真值检测，显式回退）
  camera  color_id.source=camera  seeds 42-46   HSV 相机检测臂（默认感知链）

判定口径（计划 P3）：两臂全 PASS、col=0、done_t 在方差带内、minclr 不降。

每个 cell：patch 两 yaml → bash run_verify.sh → 解析 /tmp/zx2026_verify.txt +
最新 run_logs/nav_metrics_*.yaml → 归档 run_logs/matrix_<ts>/<tag>_seed<seed>/ →
追加 matrix_results.csv。含 wall-monotonic 漂移检测（>30s = 睡眠污染，留档
__contaminated 并自动重跑，沿用 matrix_run.py 纪律）。结束恢复 source=truth。

用法: python3 ~/zx2026_arena_ws/tools/cam_matrix_run.py
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
CFG = WS + "/src/zx2026_common/config/sim_settings.yaml"
RULES = WS + "/src/zx2026_common/config/competition_rules.yaml"
VERIFY_TXT = "/tmp/zx2026_verify.txt"
SMOKE_LOG = "/tmp/zx2026_smoke.log"

SEEDS = (42, 43, 44, 45, 46)   # 与 matrix_run.py 五 seed 口径一致；argv 可覆写


def build_cells(seeds):
    cells = []
    for seed in seeds:
        cells.append(dict(tag="truth", seed=seed, source="truth"))
        cells.append(dict(tag="camera", seed=seed, source="camera"))
    return cells


def _time_limit_s():
    try:
        with open(RULES) as f:
            return float(yaml.safe_load(f).get("time_limit_s", 600.0))
    except Exception:
        return 600.0


VERIFY_TIMEOUT = _time_limit_s() + 300.0


def set_top(path, key, value):
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    for i, ln in enumerate(lines):
        if re.match(r"^%s\s*:" % key, ln):
            lines[i] = re.sub(r"^(%s\s*:\s*)\S+" % key,
                              lambda m: m.group(1) + str(value), ln, count=1)
            break
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)


def set_key_in_block(path, block, key, value):
    """块内首个 `key:` 的值改成 str(value)（保留行内注释，块外零改动）。

    块范围界定：遇到缩进 <= 块键缩进的实体行即止；注释行跳过。
    只改第一个匹配（color_id.source 在块内首键，子块无同名键，安全）。
    """
    with open(path, encoding="utf-8") as f:
        lines = f.readlines()
    blk_indent = None
    for i, ln in enumerate(lines):
        m = re.match(r"^(\s*)%s\s*:" % block, ln)
        if m:
            blk_indent = len(m.group(1))
            continue
        if blk_indent is not None:
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            indent = len(ln) - len(ln.lstrip())
            if indent <= blk_indent:
                break
            if re.match(r"%s\s*:" % key, s):
                lines[i] = re.sub(r"^(\s*%s\s*:\s*)\S+" % key,
                                  lambda mm: mm.group(1) + str(value), ln, count=1)
                break
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)


def parse_metrics(path):
    d = yaml.safe_load(open(path)) or {}
    drones = d.get("drones", {})
    agg = dict(col=0, stuck_s=0.0, stuck_n=0, flips=0, dist=0.0, minclr=1e9)
    for v in drones.values():
        agg["col"] += int(v.get("collisions", 0))
        agg["stuck_s"] += float(v.get("stuck_s", 0))
        agg["stuck_n"] += int(v.get("stuck_n", 0))
        agg["flips"] += int(v.get("flips", 0))
        agg["dist"] += float(v.get("dist_m", 0))
        agg["minclr"] = min(agg["minclr"], float(v.get("min_clear_m", 1e9)))
    agg["minclr"] = round(agg["minclr"], 3) if agg["minclr"] < 1e9 else -1
    return agg


def _run_once(cell, outdir):
    tag, seed, source = cell["tag"], cell["seed"], cell["source"]
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    set_top(CFG, "run_seed", seed)
    set_key_in_block(RULES, "color_id", "source", source)

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
    return row, drift


def run_cell(cell, outdir):
    row, drift = _run_once(cell, outdir)
    if drift > 30.0:
        print("    !! wall-monotonic 漂移 %.0fs —— 运行中断档/睡眠，cell 污染："
              "留档 __contaminated，自动重跑" % drift, flush=True)
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
    outroot = os.path.join(WS, "run_logs", "matrix_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = RULES + ".cam_bak"
    shutil.copy(RULES, backup)
    backup_cfg = CFG + ".cam_bak"
    shutil.copy(CFG, backup_cfg)
    csv_path = os.path.join(outroot, "matrix_results.csv")
    cols = ["tag", "seed", "verdict", "score", "done_t", "col", "stuck_s",
            "stuck_n", "flips", "dist", "minclr", "runtime_s", "sleep_s",
            "timeout"]
    print("cam A/B matrix start ->", outroot, flush=True)
    seeds = [int(a) for a in sys.argv[1:]] or list(SEEDS)
    cells = build_cells(seeds)
    print("cells:", [(c["tag"], c["seed"]) for c in cells], flush=True)
    rows = []
    try:
        for k, cell in enumerate(cells):
            name = "%02d_%s_seed%d" % (k, cell["tag"], cell["seed"])
            print("[%d/%d] %s running..." % (k + 1, len(cells), name), flush=True)
            row = run_cell(cell, os.path.join(outroot, name))
            rows.append(row)
            with open(csv_path, "a") as f:
                if k == 0:
                    f.write(",".join(cols) + "\n")
                f.write(",".join(str(row[c]) for c in cols) + "\n")
            print("    -> %s score=%s done=%ss col=%s stuck=%.1fs flips=%s "
                  "minclr=%s (%.0fs)"
                  % (row["verdict"], row["score"], row["done_t"], row["col"],
                     row["stuck_s"], row["flips"], row["minclr"],
                     row["runtime_s"]), flush=True)
    finally:
        shutil.copy(backup_cfg, CFG)   # run_seed 亦须还原（否则残留最后 cell 值）
        os.remove(backup_cfg)
        shutil.copy(backup, RULES)
        os.remove(backup)
        set_key_in_block(RULES, "color_id", "source", "camera")  # 默认=感知链

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
