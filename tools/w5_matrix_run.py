#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w5_matrix_run.py — W5 双旗 A/B 单 cell 跑批。

用法: python3 tools/w5_matrix_run.py <tag> <seed> <sign_fix:0|1> <fence:0|1>
patch sim_settings.yaml（run_seed + closed_loop.cloud_avoidance.sign_fix +
collision.recovery.fence_events，只动值不动注释）→ bash run_verify_gazebo.sh →
归档 run_logs/w5_<ts>/<tag>_seed<seed>/（verify.txt + 最新 nav_metrics + nav_node
日志 + collision_monitor 日志）→ 恢复原始配置。摘要含 STALL/CG/围栏碰撞计数。
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


def patch_cfg(text, seed, sign_fix, fence):
    text = re.sub(r"(?m)^(run_seed:)\s*\d+", lambda m: "%s %d" % (m.group(1), seed), text)
    p1 = re.compile(r"(?m)^(  cloud_avoidance:\n    sign_fix: )\w+")
    text, n1 = p1.subn(lambda m: m.group(1) + ("true" if sign_fix else "false"), text)
    p2 = re.compile(r"(?m)^(    fence_events: )\w+")
    text, n2 = p2.subn(lambda m: m.group(1) + ("true" if fence else "false"), text)
    if n1 != 1 or n2 != 1:
        raise RuntimeError("patch hits sign_fix=%d fence_events=%d" % (n1, n2))
    return text


def main():
    tag, seed = sys.argv[1], int(sys.argv[2])
    sign_fix, fence = sys.argv[3] == "1", sys.argv[4] == "1"
    nav_py = WS + "/src/arena_nav/scripts/nav_node.py"
    mon_py = WS + "/src/arena_world_gazebo/scripts/gazebo_collision_monitor.py"
    for p in (nav_py, mon_py):
        if not os.access(p, os.X_OK):
            raise RuntimeError("not executable: %s — chmod +x first" % p)
    with open(CFG) as f:
        orig = f.read()
    outdir = os.path.join(RUNLOGS, "w5_%s" % time.strftime("%Y%m%d_%H%M%S"),
                          "%s_seed%d" % (tag, seed))
    os.makedirs(outdir, exist_ok=True)
    t0 = time.time()
    try:
        with open(CFG, "w") as f:
            f.write(patch_cfg(orig, seed, sign_fix, fence))
        print("[cell] %s seed=%d sign_fix=%s fence=%s -> run_verify_gazebo.sh"
              % (tag, seed, sign_fix, fence))
        subprocess.run(["bash", WS + "/run_verify_gazebo.sh"],
                       timeout=2100, check=False)
    finally:
        with open(CFG, "w") as f:
            f.write(orig)
        print("[cell] config restored")

    if os.path.exists(VERIFY_TXT):
        shutil.copy(VERIFY_TXT, os.path.join(outdir, "verify.txt"))
    ymls = [p for p in glob.glob(RUNLOGS + "/nav_metrics_2*.yaml")]
    latest = max(ymls, key=os.path.getmtime) if ymls else None
    if latest and os.path.getmtime(latest) < t0:
        print("[cell] WARN stale metrics %s -> ignore" % os.path.basename(latest))
        latest = None
    if latest:
        shutil.copy(latest, os.path.join(outdir, "nav_metrics.yaml"))
    navlogs = glob.glob(ROSLOG + "/nav_node_*.log")
    for p in navlogs:
        shutil.copy(p, os.path.join(outdir, os.path.basename(p)))
    monlogs = glob.glob(ROSLOG + "/gazebo_collision_monitor*.log")
    for p in monlogs:
        shutil.copy(p, os.path.join(outdir, os.path.basename(p)))

    print("[cell] wall=%.0fs outdir=%s" % (time.time() - t0, outdir))
    vtxt = os.path.join(outdir, "verify.txt")
    if os.path.exists(vtxt):
        with open(vtxt, errors="replace") as f:
            for line in f:
                if line.startswith("VERDICT:"):
                    print("[cell] %s" % line.strip())
    if os.path.exists("/tmp/zx2026_gazebo_smoke.log"):
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
    cg = stall = col_evt = 0
    for p in navlogs:
        with open(p, errors="replace") as f:
            for line in f:
                if "COLLAPSE-GUARD" in line:
                    cg += 1
                elif "STALL diag" in line:
                    stall += 1
    for p in monlogs:
        with open(p, errors="replace") as f:
            for line in f:
                if "collision" in line:
                    col_evt += 1
    print("[cell] CG_lines=%d STALL_lines=%d mon_collision_lines=%d"
          % (cg, stall, col_evt))


if __name__ == "__main__":
    main()
