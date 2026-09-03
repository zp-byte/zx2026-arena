#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""W1 matrix 法证：逐 FAIL cell 的 per-drone 分数 + 证据行提取。"""
import os
import re

import yaml

D = os.path.expanduser(
    "~/zx2026_arena_ws/run_logs/matrix_w1_20260903_185635")
CELLS = ["01_B_vs_seed42", "03_D_full_seed42", "07_D_full_seed43",
         "10_C_vs_rq_seed45", "11_D_full_seed45", "00_A_base_seed42",
         "08_A_base_seed45", "05_B_vs_seed43", "09_B_vs_seed45",
         "06_C_vs_rq_seed43"]

for c in CELLS:
    sp = os.path.join(D, c, "score.yaml")
    ep = os.path.join(D, c, "evidence.txt")
    print("== %s ==" % c)
    if not os.path.exists(sp):
        print("  (no score.yaml)")
        continue
    d = yaml.safe_load(open(sp))
    for k, v in sorted(d["scores"].items(), key=lambda x: int(x[0])):
        if v["score"] < 43 or not v["crossed_zone"]:
            print("  drone%s: cross=%s match=%s path=%.1f score=%s"
                  % (k, v["crossed_zone"], v["match"], v["path_length"],
                     v["score"]))
    print("  total: %s" % d["total"])
    # 该 cell 内 VIA-SLOT 选槽 + 每 drone 事件计数
    if os.path.exists(ep):
        txt = open(ep, encoding="utf-8", errors="replace").read()
        for m in re.finditer(r"drone (\d+) VIA-SLOT \(([-\d.]+),([-\d.]+)\)"
                             r" clr=([-\d.]+)", txt):
            print("  VIA-SLOT d%s=(%s,%s) clr=%s" % m.groups())
        nd = {}
        for ln in txt.splitlines():
            m = re.match(r"\S*\s*nav_node\.py[^:]*:\s*.*drone (\d)", ln)
            if not m:
                m = re.search(r"drone (\d)", ln)
            if m:
                nd[m.group(1)] = nd.get(m.group(1), 0) + 1
        print("  evidence lines per drone: %s"
              % dict(sorted(nd.items())))
        # 最后一台没过带的机的关键事件
        zeros = [k for k, v in d["scores"].items() if not v["crossed_zone"]]
        for z in zeros:
            pats = ["drone %s " % z]
            hit = [ln.split("|", 1)[1].strip()[:110] for ln in txt.splitlines()
                   if any(p in ln for p in pats)
                   and re.search(r"VIA-SLOT|DEAD-END|escape done|goal-seal|"
                                 r"quiet window|STALL|fallback", ln)]
            print("  drone%s key events (%d):" % (z, len(hit)))
            for h in hit[:14]:
                print("    " + h)
