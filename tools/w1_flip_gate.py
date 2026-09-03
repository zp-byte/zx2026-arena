#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""翻默认 smoke 证据门：旗开形态断言（与 Phase 0 旗关门互为镜像）。"""
import glob
import os
import re
import sys

ROS_LOG = os.path.expanduser("~/.ros/log")
dirs = [d for d in glob.glob(os.path.join(ROS_LOG, "*"))
        if os.path.isdir(d) and os.path.basename(d) != "latest"]
run_dir = max(dirs, key=os.path.getmtime)
txt = ""
for fp in sorted(glob.glob(os.path.join(run_dir, "*.log"))):
    try:
        txt += open(fp, encoding="utf-8", errors="replace").read()
    except OSError:
        pass

ok = True


def check(cond, msg):
    global ok
    print(("  OK: " if cond else "  FAIL: ") + msg)
    if not cond:
        ok = False


print("run_dir=%s" % os.path.basename(run_dir))
m = re.search(r"soa=(\w+) swg=(\w+) swq=(\w+)", txt)
check(m is not None, "banner (soa=/swg=/swq=) found")
if m:
    check((m.group(1), m.group(2), m.group(3)) == ("True", "True", "True"),
          "banner flags = soa=True swg=True swq=True (got %s)" % m.group(0))

slots = set(re.findall(r"drone (\d) VIA-SLOT \(([-\d.]+),([-\d.]+)\)", txt))
check(len(slots) == 6, "VIA-SLOT x6 assigned (got %d: %s)"
      % (len(slots), sorted(slots)))
check("via_slots 无可行槽位集" not in txt, "zero slot fallback")

n_swarm = len(re.findall(r"swarm_guard hit", txt))
n_sep = len(re.findall(r"sep_guard hit", txt))
print("  info: sep_guard hits=%d swarm_guard hits=%d" % (n_sep, n_swarm))

n_quiet = len(re.findall(r"swarm quiet window", txt))
n_de = len(re.findall(r"DEAD-END escape engaged", txt))
n_de_done = len(re.findall(r"dead-end escape done", txt))
print("  info: de engaged=%d done=%d quiet windows=%d" % (n_de, n_de_done, n_quiet))

n_col = len(re.findall(r"collision marker at", txt))
if n_col:
    check("close=" in txt, "collision lines carry close= telemetry")
print("  info: collisions=%d" % n_col)

verify = open("/tmp/zx2026_verify.txt").read()
check("VERDICT: PASS" in verify, "verify verdict PASS")
m2 = re.search(r"score_summary: total=(\d+)", verify)
check(m2 is not None, "score total present")
if m2:
    print("  info: total=%s" % m2.group(1))
zeros = re.findall(r"(\d+):\s*\n\s+crossed_zone: false", verify)
check(not zeros, "no drone missed the zone (got %s)" % zeros)

print("FLIP-GATE: %s" % ("PASS" if ok else "FAIL"))
sys.exit(0 if ok else 2)
