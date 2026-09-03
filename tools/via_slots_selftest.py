#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""W1 via_slots selftest：真实场景槽位可行性 + 确定性 + 种子敏感性 + 失败回退。

真实森林 density_seed=42 固定（scene_topology.yaml，不随 run_seed 变）→
槽位集合全局确定，验证一次即代表全部 A/B seed。
运行: source /opt/ros/noetic/setup.bash && source ~/zx2026_arena_ws/devel/setup.bash \
      && python3 tools/via_slots_selftest.py
"""
import copy
import math
import os
import sys

WS = os.path.expanduser("~/zx2026_arena_ws")
sys.path.insert(0, os.path.join(WS, "src/arena_mission/scripts"))

from zx2026_common import config as cfg
from zx2026_common.scene import Scene
from mission_executor_node import select_via_slots, _slot_clearance

N = 0


def ok(cond, msg):
    global N
    N += 1
    if not cond:
        raise AssertionError("case %d FAIL: %s" % (N, msg))


# V1 真实森林：6 槽全可行（zone 内 / 净空 >= slot_clear / 互距 / y 升序）
scene = Scene()
slots = select_via_slots(scene, scene.drone_count, z_lo=2.0, z_hi=3.0)
ok(slots is not None, "real forest must yield a feasible slot set")
ok(len(slots) == scene.drone_count,
   "slot count == drone_count (%s vs %s)" % (len(slots), scene.drone_count))
for (sx, sy, clr) in slots:
    ok(scene.in_crossing_zone((sx, sy)), "slot (%.2f,%.2f) in zone" % (sx, sy))
    ok(clr >= 1.5, "slot (%.2f,%.2f) clearance %.3f >= 1.5" % (sx, sy, clr))
    ok(abs(_slot_clearance(scene, sx, sy, 2.0, 3.0) - clr) < 1e-9,
       "slot (%.2f,%.2f) clearance value consistent" % (sx, sy))
min_d, min_dy = 1e9, 1e9
for i in range(len(slots)):
    for j in range(i + 1, len(slots)):
        d = math.hypot(slots[i][0] - slots[j][0], slots[i][1] - slots[j][1])
        dy = abs(slots[i][1] - slots[j][1])
        min_d = min(min_d, d)
        min_dy = min(min_dy, dy)
        ok(d >= 1.8, "pairwise dist %.2f >= 1.8 (ladder floor)" % d)
ok(all(slots[i][1] <= slots[i + 1][1] for i in range(len(slots) - 1)),
   "slots y-ascending")

# V1m zone_margin 不变量（matrix_w1 教训回归）：每槽距带缘 >= zone_margin(1.0)
# > drop_tol(0.8)，保证 executor CROSS_ZONE 腿"到槽⇒过带"，绊线永不误杀
zxs = [p[0] for p in scene.crossing_zone]
zys = [p[1] for p in scene.crossing_zone]
for (sx, sy, _) in slots:
    ok(min(zxs) + 1.0 <= sx <= max(zxs) - 1.0
       and min(zys) + 1.0 <= sy <= max(zys) - 1.0,
       "slot (%.2f,%.2f) >= zone_margin 1.0 inside zone (drop_tol ball "
       "contained)" % (sx, sy))

# V1b y 分带覆盖（编队前散开意图）：zone_margin 内缩收窄候选带后允许 2 带借空
# （散开性由逐对 sep 保证；V1 已锁 6 槽全可行）
band_h = (max(zys) - min(zys)) / scene.drone_count
occ = len({min(int((sy - min(zys)) / band_h), scene.drone_count - 1)
           for (_, sy, _) in slots})
ok(occ >= scene.drone_count - 2,
   "band coverage %d/%d (front spread, margin-shrunk)" % (occ, scene.drone_count))

# V2 热点树（穿越区重心最近树 = tree#24 (1.1,2.0)）：每槽对其净空 >= slot_clear
cz = scene.crossing_zone_center
t24 = min((ob for ob in scene.obstacles if ob.kind == "tree"),
          key=lambda ob: math.hypot(ob.cx - cz[0], ob.cy - cz[1]))
for (sx, sy, clr) in slots:
    d24 = math.hypot(sx - t24.cx, sy - t24.cy) - t24.trunk_r
    ok(d24 >= 1.5,
       "slot (%.2f,%.2f) clear of hotspot tree (%.2f)" % (sx, sy, d24))

# V3 确定性：两次调用逐位一致
slots2 = select_via_slots(scene, scene.drone_count, z_lo=2.0, z_hi=3.0)
ok(slots == slots2, "deterministic selection")

# V4 种子敏感性：density_seed 43/45 场景注入后仍可行（回退梯子允许参与）
topo = cfg.load("scene_topology.yaml")
for seed in (43, 45):
    topo2 = copy.deepcopy(topo)
    hit = False
    for z in topo2.get("zones", []):
        if isinstance(z, dict) and z.get("kind") == "forest" and "forest" in z:
            z["forest"]["density_seed"] = seed
            hit = True
    ok(hit, "patched forest density_seed=%d" % seed)
    sc2 = Scene(scene_cfg=topo2)
    s2 = select_via_slots(sc2, sc2.drone_count, z_lo=2.0, z_hi=3.0)
    ok(s2 is not None and len(s2) == sc2.drone_count,
       "seed %d feasible (%s)" % (seed, "None" if s2 is None else len(s2)))

# V5 不可行净空 → None（调用方回退共享越界点原行为）
ok(select_via_slots(scene, scene.drone_count, slot_clear=50.0) is None,
   "infeasible slot_clear returns None")

# V6 drop_y 名次映射单调（槽位 y 升序 ↔ 名次升序，保 straightness）
keys = sorted((dp.xyz[1], dp.xyz[0]) for dp in scene.drop_points)
ok(len(keys) <= len(slots), "every drop point has a slot")
rank_ys = [slots[i][1] for i in range(len(keys))]
ok(rank_ys == sorted(rank_ys), "rank->slot y monotonic")

print("via_slots_selftest: all %d cases PASS" % N)
print("hotspot tree: (%.2f, %.2f) trunk_r=%.2f" % (t24.cx, t24.cy, t24.trunk_r))
print("seed42 slots (min pairwise dist=%.2f, min |dy|=%.2f, bands=%d/%d):"
      % (min_d, min_dy, occ, scene.drone_count))
for i, (sx, sy, clr) in enumerate(slots):
    print("  slot %d: (%.2f, %.2f) clr=%.2f" % (i, sx, sy, clr))
