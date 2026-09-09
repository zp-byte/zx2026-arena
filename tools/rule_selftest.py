#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rule_selftest.py — 科目三规则链纯函数自检（无 ROS，P8 收口第 1 关）。

仿 vo_selftest 风格：A/B/C/D 四组锚点全过 exit 0，任一 FAIL exit 1。
  A bucket_select  命中/异色清零/漏检清零/低置信冻结/锁定幂等/全扫无果
  B rules          oob 计时 grace 进退、计划内落地豁免、pad 外贴地防抖、
                   高度 grace、corridor_hit、geofence⊂围栏 AABB、起降区⊂geofence
  C scoring        judge_drop 四态边界（0.6±ε）、wrong_color 优先、s1 封顶、s2 七档
  D 一致性         fleet.payload ↔ color_map ↔ drop_points 三方一致、
                   3 平台 3 色 3 型不重不漏、task_generator i%3 契约复演

用法： python3 tools/rule_selftest.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "src", "zx2026_common", "scripts"))

from zx2026_common import config as cfg              # noqa: E402
from zx2026_common import rules, scoring             # noqa: E402
from zx2026_common import bucket_select as bs        # noqa: E402
from zx2026_common.scene import Scene                # noqa: E402

RED, BLUE, YELLOW = 0, 1, 2  # cfg.COLOR_TO_UINT8 同序

_results = []


def check(name, ok):
    _results.append((name, bool(ok)))
    print("[%s] %s" % ("PASS" if ok else "FAIL", name))


# ---------------------------------------------------------------- A bucket_select
def group_a():
    st, d = bs.bucket_scan_step(RED, 0.9, RED, state=None)
    check("A1 命中计数 1/3", d is None and st["hit"] == 1)
    st, d = bs.bucket_scan_step(RED, 0.9, RED, state=st)
    st, d = bs.bucket_scan_step(RED, 0.9, RED, state=st)
    check("A2 连续 3 命中 LOCK", d == "LOCK")
    st2, d2 = bs.bucket_scan_step(RED, 0.9, RED, state=st)
    check("A3 锁定后幂等继续 LOCK", d2 == "LOCK")
    st, _ = bs.bucket_scan_step(BLUE, 0.9, RED, state=st)
    check("A4 异色清零", st["hit"] == 0)
    st, _ = bs.bucket_scan_step(bs.NO_DET, 0.0, RED, state=st)
    check("A5 漏检清零", st["hit"] == 0)
    st = {"hit": 2}
    st, _ = bs.bucket_scan_step(RED, 0.5, RED, state=st)
    check("A6 低置信冻结（不计不清）", st["hit"] == 2)
    st, _ = bs.bucket_scan_step(RED, 0.9, RED, state=st)
    check("A7 冻结后命中即达 3 锁", _ == "LOCK")
    # 全扫无果是 executor 层决策（dwell 耗尽推进），纯函数层只保证
    # 异色/漏检永不 LOCK——这里验证 100 拍污染序列不出假锁
    st = bs.new_state()
    seq = [(RED, 0.9), (bs.NO_DET, 0.0), (BLUE, 0.9)] * 33
    fake = False
    for c, cf in seq:
        st, d = bs.bucket_scan_step(c, cf, RED, state=st)
        if d == "LOCK":
            fake = True
    check("A8 污染序列无假锁", not fake)


# ---------------------------------------------------------------- B rules
def group_b():
    acc, trip = rules.oob_tick(True, 9.9, 10.0, dt=0.05)
    check("B1 oob 9.95s 不杀", not trip and abs(acc - 9.95) < 1e-9)
    acc, trip = rules.oob_tick(True, 9.95, 10.0, dt=0.05)
    check("B2 oob 10.0s 杀", trip)
    acc, trip = rules.oob_tick(False, 9.95, 10.0, dt=0.05)
    check("B3 oob 入界回零", acc == 0.0 and not trip)

    acc, v = rules.landing_verdict(0.2, 0.30, False, False, 0.0, 1.5, dt=0.05)
    check("B4 pad 外贴地未 armed 累计", v == "none" and abs(acc - 0.05) < 1e-9)
    acc, v = rules.landing_verdict(0.2, 0.30, False, False, 1.45, 1.5, dt=0.05)
    check("B5 贴地防抖 1.5s 判擅自落地", v == "unauthorized")
    acc, v = rules.landing_verdict(0.2, 0.30, True, True, 1.45, 1.5, dt=0.05)
    check("B6 armed+pad 内=计划内 touchdown", v == "planned")
    acc, v = rules.landing_verdict(0.2, 0.30, True, False, 0.0, 1.5, dt=0.05)
    check("B7 armed 但 pad 外不判 planned", v == "none" and acc == 0.0)
    acc, v = rules.landing_verdict(0.5, 0.30, False, False, 1.0, 1.5, dt=0.05)
    check("B8 离地回零", acc == 0.0 and v == "none")

    acc, trip = rules.height_verdict(5.1, 5.0, 1.9, 2.0, dt=0.05)
    check("B9 超高 1.95s 不杀", not trip)
    acc, trip = rules.height_verdict(5.1, 5.0, 1.95, 2.0, dt=0.05)
    check("B10 超高 2.0s 杀", trip)
    acc, trip = rules.height_verdict(4.9, 5.0, 1.95, 2.0, dt=0.05)
    check("B11 降高回零", acc == 0.0 and not trip)

    poly = [(0, 0), (10, 0), (10, 10), (0, 10)]
    check("B12 corridor_hit 命中",
          rules.corridor_hit([(20, 20), (5, 5)], poly))
    check("B13 corridor_miss 不命中",
          not rules.corridor_hit([(20, 20), (25, 25)], poly))
    check("B14 空轨迹不命中", not rules.corridor_hit([], poly))

    # geofence ⊂ 围栏 AABB（world_builder fence_e 18.6 vs yaml 22.5 教训）
    scene = Scene()
    fences = [o for o in scene.obstacles if o.kind == "fence"]
    check("B15 场内存在围栏障碍", len(fences) > 0)
    aabb = (min(o.lo[0] for o in fences), max(o.hi[0] for o in fences),
            min(o.lo[1] for o in fences), max(o.hi[1] for o in fences))
    rc = cfg.load("competition_rules.yaml").get("rule", {})
    gf = [(float(p[0]), float(p[1]))
          for p in (rc.get("geofence") or [])]
    # 场地西侧无物理围栏（开放=起降区），geofence 西界故意外扩盖住 pads——
    # 只断言东/北/南三界被物理围栏覆盖（rule_monitor._assert_fences 同语义）
    check("B16 geofence 东/北/南三界 ⊂ 围栏（西侧开放=起降区）",
          len(gf) == 4
          and max(p[0] for p in gf) <= aabb[1] + 1e-6
          and max(p[1] for p in gf) <= aabb[3] + 1e-6
          and min(p[1] for p in gf) >= aabb[2] - 1e-6)
    pad = [(float(p[0]), float(p[1]))
           for p in scene.zones["takeoff"].polygon]
    xs = [p[0] for p in gf]
    ys = [p[1] for p in gf]
    check("B17 起降区 ⊂ geofence（西界含 pads 教训）",
          all(min(xs) <= p[0] <= max(xs) and min(ys) <= p[1] <= max(ys)
              for p in pad))


# ---------------------------------------------------------------- C scoring
def group_c():
    v = scoring.judge_drop((19.5, -6.0), (19.5, -6.0), "red", "red", 0.6)
    check("C1 平台心释放 correct", v == scoring.VERDICT_CORRECT)
    v = scoring.judge_drop((19.5 + 0.59, -6.0), (19.5, -6.0), "red", "red", 0.6)
    check("C2 0.59m 边内 correct", v == scoring.VERDICT_CORRECT)
    v = scoring.judge_drop((19.5 + 0.61, -6.0), (19.5, -6.0), "red", "red", 0.6)
    check("C3 0.61m 边外 off_bucket", v == scoring.VERDICT_OFF_BUCKET)
    v = scoring.judge_drop((19.5, -6.5), (19.5, -6.0), "red", "blue", 0.6)
    check("C4 异色 wrong_color 优先于偏平台", v == scoring.VERDICT_WRONG_COLOR)
    v = scoring.judge_drop((19.5, -6.0), (19.5, -6.0), "red", "blue", 0.6)
    check("C5 异色+平台心仍 wrong_color", v == scoring.VERDICT_WRONG_COLOR)
    d = [(scoring.VERDICT_CORRECT)] * 6
    check("C6 六正确 s1=60", scoring.s1_team(d) == 60)
    d = [scoring.VERDICT_CORRECT] * 7
    check("C7 封顶 60", scoring.s1_team(d) == 60)
    d = [scoring.VERDICT_CORRECT, scoring.VERDICT_WRONG_COLOR,
         scoring.VERDICT_OFF_BUCKET]
    check("C8 错投 0 分不倒扣", scoring.s1_team(d) == 10)
    ladder = {6: 40, 5: 30, 4: 25, 3: 15, 2: 10, 1: 5, 0: 0}
    check("C9 s2 七档", [scoring.s2_from_landed(n, ladder)
                         for n in range(7)] == [0, 5, 10, 15, 25, 30, 40])
    check("C10 s2 越界档回 0", scoring.s2_from_landed(9, ladder) == 0)


# ---------------------------------------------------------------- D 一致性
def group_d():
    scene = Scene()
    fleet = cfg.load("fleet.yaml").get("drones", [])
    topo = cfg.load("scene_topology.yaml")
    cmap = topo.get("color_map", {})
    dps = scene.drop_points

    check("D1 投送点=3（规则口径）", len(dps) == 3)
    types = sorted(dp.type_id for dp in dps)
    check("D2 3 型不重不漏", types == ["TYPE_A", "TYPE_B", "TYPE_C"])
    colors = sorted(dp.color for dp in dps)
    check("D3 3 色不重不漏", colors == ["blue", "red", "yellow"])
    check("D4 平台色==color_map[type]",
          all(dp.color == cmap.get(dp.type_id) for dp in dps))
    check("D5 drop_points 邻平台间距 ≥ 4m（平台 1.2m 投影不重叠）",
          all(math.hypot(a.xyz[0] - b.xyz[0], a.xyz[1] - b.xyz[1]) >= 4.0
              for i, a in enumerate(dps) for b in dps[i + 1:]))

    check("D6 机数=6 且 fleet payload=TYPE_[ABC] 各 2",
          len(fleet) == scene.drone_count == 6
          and sorted(d["payload"] for d in fleet)
          == ["TYPE_A", "TYPE_A", "TYPE_B", "TYPE_B", "TYPE_C", "TYPE_C"])
    # task_generator 契约：payload = TYPE_[ABC][i%3] → box_color=color_for_type
    ok = True
    for i in range(scene.drone_count):
        pt = "TYPE_" + "ABC"[i % 3]
        if fleet[i]["payload"] != pt or cmap.get(pt) != scene.color_for_type(pt):
            ok = False
    check("D7 task_generator i%3 契约三方一致", ok)
    check("D8 色编码完备", set(cfg.COLOR_TO_UINT8) == {"red", "blue", "yellow",
                                                       "UNKNOWN"})


def main():
    print("== rule_selftest (P8) ==")
    group_a()
    group_b()
    group_c()
    group_d()
    fails = [n for n, ok in _results if not ok]
    print("== %d/%d PASS ==" % (len(_results) - len(fails), len(_results)))
    if fails:
        print("FAILED:", "; ".join(fails))
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
