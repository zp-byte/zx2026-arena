#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import glob
import os
import re
import sys

sys.path.insert(0, "/home/ubuntu/zx2026_arena_ws/tools")
import w2_phase_attr as W  # noqa: E402

md = sorted(glob.glob("/home/ubuntu/zx2026_arena_ws/run_logs/matrix_w1_*"),
            key=os.path.getmtime)
print("matrix dirs:", [os.path.basename(m) for m in md])
for m in md:
    evs = sorted(glob.glob(os.path.join(m, "*", "evidence.txt")))
    print(" %s: %d cells" % (os.path.basename(m), len(evs)))
    if not evs:
        continue
    ev = evs[0]
    rd = W.run_dir_of(ev)
    print("   first cell %s -> run_dir %s (exists=%s)"
          % (os.path.basename(os.path.dirname(ev)), rd, rd is not None))
    if not rd:
        continue
    navs = [f for f in glob.glob(os.path.join(rd, "nav_node_*-*.log"))
            if "-stdout" not in f]
    print("   nav_node logs: %d" % len(navs))
    n_hit = 0
    for fp in navs:
        for ln in open(fp, encoding="utf-8", errors="replace"):
            if "collision marker at" in ln:
                n_hit += 1
                if n_hit <= 2:
                    print("   sample:", repr(ln.strip()[:150]))
                    print("   COL_RE match:",
                          bool(W.COL_RE.search(ln)),
                          "ROSOUT:", bool(W.COL_RE_ROSOUT.search(ln)))
    print("   raw collision lines: %d" % n_hit)
    rows = W.attr_run(rd)
    print("   attr_run rows: %d" % len(rows))
