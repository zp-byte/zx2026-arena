#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""zone_margin 修复后的槽位表快查（三 seed + 不变量断言）。"""
import copy
import sys

sys.path.insert(0, "/home/ubuntu/zx2026_arena_ws/src/arena_mission/scripts")
from mission_executor_node import select_via_slots  # noqa: E402
from zx2026_common.scene import Scene  # noqa: E402

import zx2026_common.config as cfgmod  # noqa: E402
topo = cfgmod.load("scene_topology.yaml")

ok = True
for seed in (42, 43, 45):
    z = copy.deepcopy(topo)
    hit = False
    for zone in z.get("zones", []):
        if isinstance(zone, dict) and zone.get("kind") == "forest" \
                and "forest" in zone:
            zone["forest"]["density_seed"] = seed
            hit = True
    assert hit, "no forest zone patched"
    sc = Scene(scene_cfg=z)
    slots = select_via_slots(sc, 6, slot_clear=1.5, slot_sep=2.5,
                             z_lo=2.0, z_hi=3.0, zone_margin=1.0)
    print("seed %d: %s" % (seed, slots))
    if slots is None:
        ok = False
        continue
    poly = sc.crossing_zone
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    m = 0.8  # drop_tol
    for px, py, clr in slots:
        # 不变量：drop_tol 球全含带内（矩形 → 逐轴 margin）
        if not (min(xs) + m <= px <= max(xs) - m
                and min(ys) + m <= py <= max(ys) - m):
            print("  VIOLATION: slot (%.2f,%.2f) drop_tol ball pokes out" % (px, py))
            ok = False
        if clr < 1.5:
            print("  VIOLATION: clr %.2f < 1.5" % clr)
            ok = False
    for i in range(len(slots)):
        for j in range(i + 1, len(slots)):
            import math
            d = math.hypot(slots[i][0] - slots[j][0], slots[i][1] - slots[j][1])
            if d < 1.8:
                print("  VIOLATION: pair dist %.2f < 1.8" % d)
                ok = False

print("MARGIN-CHECK: %s" % ("PASS" if ok else "FAIL"))
