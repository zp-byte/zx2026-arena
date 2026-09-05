#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/cg_matrix_run.py — W3 塌缩卫兵 A/B 单 cell 跑批。

用法: python3 tools/cg_matrix_run.py <tag> <seed> <guard:0|1>
每个 cell：patch sim_settings.yaml（run_seed + collapse_guard.enabled，只动值
不动注释）→ bash run_verify_gazebo.sh（Gazebo 后端全栈+端到端任务）→ 解析
/tmp/zx2026_verify.txt + 最新 run_logs/nav_metrics_*.yaml → 归档
run_logs/cg_<ts>/<tag>_seed<seed>/（含 ~/.ros/log/latest 的 nav_node 日志）→
恢复原始配置。结束打印单 cell 摘要（state/sim_t/stuck/dist/col + COLLAPSE
-GUARD / STALL 触发计数）。
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
RUNLOGS = WS + "/run_logs"
VERIFY_TXT = "/tmp/zx2026_verify.txt"
ROSLOG = os.path.expanduser("~/.ros/log/latest")


def patch_cfg(text, seed, guard):
    text = re.sub(r"(?m)^(run_seed:)\s*\d+", lambda m: "%s %d" % (m.group(1), seed), text)
    pat = re.compile(r"(?m)^(  collapse_guard:\n    enabled: )\w+")
    text, n = pat.subn(lambda m: m.group(1) + ("true" if guard else "false"),
                       text)
    if n != 1:
        raise RuntimeError("collapse_guard.enabled patch hit %d times" % n)
    return text


def main():
    tag, seed, guard = sys.argv[1], int(sys.argv[2]), sys.argv[3] == "1"
    # 预检：nav_node.py 必须可执行（Windows 侧编辑会掉执行位 → roslaunch
    # 解析不到节点，整局静默空转，B1 事故）
    nav_py = WS + "/src/arena_nav/scripts/nav_node.py"
    if not os.access(nav_py, os.X_OK):
        raise RuntimeError("nav_node.py not executable — chmod +x first")
    with open(CFG) as f:
        orig = f.read()
    outdir = os.path.join(RUNLOGS, "cg_%s" % time.strftime("%Y%m%d_%H%M%S"),
                          "%s_seed%d" % (tag, seed))
    os.makedirs(outdir, exist_ok=True)
    t0 = time.time()
    try:
        with open(CFG, "w") as f:
            f.write(patch_cfg(orig, seed, guard))
        print("[cell] %s seed=%d guard=%s -> running run_verify_gazebo.sh"
              % (tag, seed, guard))
        subprocess.run(["bash", WS + "/run_verify_gazebo.sh"],
                       timeout=2100, check=False)
    finally:
        with open(CFG, "w") as f:
            f.write(orig)
        print("[cell] config restored")

    # 归档：verify 输出 + 最新 metrics + nav_node 日志
    if os.path.exists(VERIFY_TXT):
        shutil.copy(VERIFY_TXT, os.path.join(outdir, "verify.txt"))
    ymls = [p for p in glob.glob(RUNLOGS + "/nav_metrics_2*.yaml")]
    latest = max(ymls, key=os.path.getmtime) if ymls else None
    # 陈旧守卫：metrics 必须是本局产物（mtime 晚于本 cell 起跑），否则宁缺毋滥
    if latest and os.path.getmtime(latest) < t0:
        print("[cell] WARN stale metrics %s (older than this cell) -> ignore"
              % os.path.basename(latest))
        latest = None
    if latest:
        shutil.copy(latest, os.path.join(outdir, "nav_metrics.yaml"))
    navlogs = glob.glob(ROSLOG + "/nav_node_*.log")
    for p in navlogs:
        shutil.copy(p, os.path.join(outdir, os.path.basename(p)))

    # 摘要
    print("[cell] wall=%.0fs outdir=%s" % (time.time() - t0, outdir))
    vtxt = os.path.join(outdir, "verify.txt")
    if os.path.exists(vtxt):
        with open(vtxt, errors="replace") as f:
            for line in f:
                if line.startswith("VERDICT:"):
                    print("[cell] %s" % line.strip())
    with open("/tmp/zx2026_gazebo_smoke.log", errors="replace") as f:
        n_err = sum(1 for line in f if "cannot launch node" in line)
    print("[cell] launch_errors=%d" % n_err)
    if latest:
        with open(latest) as f:
            m = yaml.safe_load(f)
        print("[cell] state=%s sim_t=%s" % (m.get("state"), m.get("sim_t")))
        for k in sorted(m.get("drones", {}), key=int):
            d = m["drones"][k]
            print("[cell] d%s col=%s stuck=%.1fs dist=%.1fm minclr=%s"
                  % (k, d.get("collisions"), d.get("stuck_s", 0.0),
                     d.get("dist_m", 0.0), d.get("min_clear_m")))
    cg = stall = 0
    for p in navlogs:
        with open(p, errors="replace") as f:
            for line in f:
                if "COLLAPSE-GUARD" in line:
                    cg += 1
                elif "STALL diag" in line:
                    stall += 1
    print("[cell] CG_lines=%d STALL_lines=%d" % (cg, stall))


if __name__ == "__main__":
    main()
