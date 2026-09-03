#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/matrix_run.py — P1 时间一致性 A/B 验证矩阵（P0 指标产出消费方）。

矩阵（交错执行，防时段漂移混淆臂间差异）：
  A_base   tc=off wind=off  seeds 42-46   基线
  B_tc     tc=on  wind=off  seeds 42-46   P1 时间一致性
  C_wind   tc=off wind=on   seeds 42-43   风场压力基线
  D_tcwind tc=on  wind=on   seeds 42-43   P1 风场压力

每个 cell：patch sim_settings.yaml（只动值，不动注释）→ bash run_verify.sh →
解析 /tmp/zx2026_verify.txt + 最新 run_logs/nav_metrics_*.yaml → 归档到
run_logs/matrix_<ts>/<tag>_seed<seed>/ → 追加 matrix_results.csv。
结束恢复原始配置。

每个 cell 自动做 wall-monotonic 漂移检测：漂移 >30s = 运行中发生断档/睡眠
（B43p3、p4-O45/O42 法证定论：满窗口慢 FAIL 全是睡眠烧掉墙钟窗口所致，
导航策略无缺陷），现场留档 <outdir>__contaminated 并自动重跑一次。

用法: python3 ~/zx2026_arena_ws/tools/matrix_run.py
"""
import glob
import os
import re
import shutil
import subprocess
import time

import yaml

WS = os.path.expanduser("~/zx2026_arena_ws")
CFG = WS + "/src/zx2026_common/config/sim_settings.yaml"
VERIFY_TXT = "/tmp/zx2026_verify.txt"
SMOKE_LOG = "/tmp/zx2026_smoke.log"

CELLS = []
for seed in (42, 43, 44, 45, 46):
    CELLS.append(dict(tag="A_base", seed=seed, tc=False, wind=False))
    CELLS.append(dict(tag="B_tc", seed=seed, tc=True, wind=False))
for seed in (42, 43):
    CELLS.append(dict(tag="C_wind", seed=seed, tc=False, wind=True))
    CELLS.append(dict(tag="D_tcwind", seed=seed, tc=True, wind=True))


def _replace_value(line, value):
    return re.sub(r"(enabled\s*:\s*)(false|true)", lambda m: m.group(1) + value,
                  line, count=1)


def set_in_block(path, block, value):
    """块内首个 enabled: 的值改 true/false（保留行内注释，块外零改动）。

    按缩进界定块范围：块键可能嵌套（如 closed_loop.temporal_consistency），
    遇到缩进 <= 块键缩进的实体行即止；注释行跳过。
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
            if re.match(r"enabled\s*:", s):
                lines[i] = _replace_value(ln, value)
                break
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)


def set_key_in_block(path, block, key, value):
    """块内首个 `key:` 的值改成 str(value)（保留行内注释，块外零改动）。

    块范围界定同 set_in_block；只改第一个匹配（deadend_escape 内键名唯一）。
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
                # \s* 容忍块内缩进（^ 单独锚行首会因缩进永远失配——p5c 首轮教训）
                lines[i] = re.sub(r"^(\s*%s\s*:\s*)\S+" % key,
                                  lambda mm: mm.group(1) + str(value), ln, count=1)
                break
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(lines)


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
    """跑一次 run_verify.sh 并收集结果；返回 (row, sleep_drift_s)。

    drift = wall 时钟推进 - monotonic 时钟推进：机器睡眠/断档只烧 wall
    （CLOCK_MONOTONIC 不含挂起时段），drift 大即为污染证据。
    """
    tag, seed = cell["tag"], cell["seed"]
    # 清理上一 cell 可能残留的节点（防 collector 双实例交错写 CSV 造成截断行）
    subprocess.run(["pkill", "-f", "nav_metrics_node.py"], capture_output=True)
    time.sleep(1)
    set_top(CFG, "run_seed", seed)
    set_in_block(CFG, "temporal_consistency", "true" if cell["tc"] else "false")
    set_in_block(CFG, "wind", "true" if cell["wind"] else "false")
    set_in_block(CFG, "deadend_escape", "true" if cell.get("de") else "false")
    set_in_block(CFG, "veto_gate", "true" if cell.get("veto") else "false")
    if cell.get("hyst") is not None:
        set_key_in_block(CFG, "deadend_escape", "exit_hyst_s", cell["hyst"])
    if cell.get("pc") is not None:
        set_key_in_block(CFG, "point_cache", "enabled",
                         "true" if cell["pc"] else "false")
    if cell.get("dam") is not None:
        set_key_in_block(CFG, "drift_aware_margin", "enabled",
                         "true" if cell["dam"] else "false")
    # 近停区/膨胀裕度/恢复仲裁：无条件设置——补丁跨 cell 累积，缺省值也
    # 必须显式落盘（否则上一 cell 的 inflation/开关会泄漏进未声明的 cell）
    set_key_in_block(CFG, "near_stop_zone", "enabled",
                     "true" if cell.get("nstop") else "false")
    set_key_in_block(CFG, "nav", "inflation", cell.get("infl", 0.4))
    set_key_in_block(CFG, "rescue_mutex", "enabled",
                     "true" if cell.get("rm") else "false")
    set_key_in_block(CFG, "bounce_corridor", "enabled",
                     "true" if cell.get("bc") else "false")
    set_key_in_block(CFG, "sep_obs_guard", "enabled",
                     "true" if cell.get("soa") else "false")
    set_key_in_block(CFG, "hotspot_inflate", "enabled",
                     "true" if cell.get("hi") else "false")

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
    return row, drift


def run_cell(cell, outdir, rerun_on_sleep=True):
    """跑一个 cell；检出睡眠/断档污染则留档并自动重跑一次。"""
    row, drift = _run_once(cell, outdir)
    if drift > 30.0 and rerun_on_sleep:
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
    outroot = os.path.join(WS, "run_logs", "matrix_" + ts)
    os.makedirs(outroot, exist_ok=True)
    backup = CFG + ".matrix_bak"
    shutil.copy(CFG, backup)
    csv_path = os.path.join(outroot, "matrix_results.csv")
    cols = ["tag", "seed", "verdict", "score", "done_t", "col", "stuck_s",
            "stuck_n", "flips", "dist", "minclr", "runtime_s", "sleep_s",
            "timeout"]
    print("matrix start ->", outroot, flush=True)
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
            print("    -> %s score=%s done=%ss col=%s stuck=%.1fs flips=%s (%.0fs)"
                  % (row["verdict"], row["score"], row["done_t"], row["col"],
                     row["stuck_s"], row["flips"], row["runtime_s"]), flush=True)
    finally:
        shutil.copy(backup, CFG)
        os.remove(backup)

    print("\n===== SUMMARY (mean per arm) =====", flush=True)
    for tag in sorted({r["tag"] for r in rows}):
        arm = [r for r in rows if r["tag"] == tag]
        n = len(arm)
        mean = lambda k: sum(r[k] for r in arm) / n
        passes = sum(1 for r in arm if r["verdict"] == "PASS")
        print("%-8s n=%d pass=%d score=%.1f done=%.0fs col=%.1f stuck=%.1fs "
              "flips=%.1f dist=%.1f"
              % (tag, n, passes, mean("score"), mean("done_t"), mean("col"),
                 mean("stuck_s"), mean("flips"), mean("dist")), flush=True)
    print("results csv:", csv_path, flush=True)


if __name__ == "__main__":
    main()
