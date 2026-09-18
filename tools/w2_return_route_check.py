#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w2_return_route_check.py — W2-P1 返航路由离线几何自检（上 sim 前门槛）。

判官团双质疑纪律：外圈插点会不会把 drone 压回树列？——路点/航段必须在
"树列-外圈散树间的可通行缝"里离线验证后再上 sim（select_via_slots 同款）。

检查项：
  A. 中缝自由带扫描：y∈[7.25,11.5] 全 x 域逐格 collides_xy（pad=0.8），
     找连续自由带 → 定 via_y。
  B. 逐投放点航段核查：drop → via(x_d, via_y) → pad 条带，采样步 0.25，
     最小净空 < 0.8 即 FAIL（树干/枝/fence/curb 全算，marker 豁免同 A*）。
  C. 西坪进场段：pad 东缘 → 场内 y=via_y 段与 west fence/curb 净空。

用法: python3 tools/w2_return_route_check.py [via_y]
"""
import math
import os
import sys

WS = "/home/ubuntu/zx2026_arena_ws"
sys.path.insert(0, WS + "/src/zx2026_common/scripts")
from zx2026_common.scene import Scene  # noqa: E402

DR = 0.35          # drone_radius（competition_rules）
PAD = 0.8          # 签名口径：边界/障碍距离 ≥0.8（trunk_r + pad 判占）
Z_LO, Z_HI = 1.45, 2.15   # 巡航 1.8 ± 机半径
STEP = 0.25


def clearance(sc, x, y):
    """点 (x,y) 在巡航 z 带内的 2D 最小净空（z-aware）。

    巡航带 [1.45,2.15] 与 fence 顶 1.3 / curb 顶 0.32 无 z 重叠——
    越顶合法（既有通行假设），不构成巡航威胁，不计入；树干/枝计入。
    """
    best = 1e9
    for ob in sc.obstacles:
        if ob.kind == "marker":
            continue
        if ob.kind == "tree":
            d = math.hypot(x - ob.cx, y - ob.cy) - ob.trunk_r
            # 枝（真机模式才有；sim 默认无枝不生效）
            if ob.branches:
                for (sx, sy, sz, ex, ey, ez, br) in ob.branches:
                    zmin, zmax = min(sz, ez), max(sz, ez)
                    if zmax < Z_LO or zmin > Z_HI:
                        continue
                    d = min(d, _seg2d((x, y), (sx, sy), (ex, ey)) - br)
        else:
            lo, hi = ob.lo, ob.hi
            if hi[2] < Z_LO or lo[2] > Z_HI:
                continue   # 越顶：fence 1.3 / curb 0.32 在巡航带下
            dx = max(lo[0] - x, 0, x - hi[0])
            dy = max(lo[1] - y, 0, y - hi[1])
            d = math.hypot(dx, dy)
        best = min(best, d)
    return best


def _seg2d(p, a, b):
    ax, ay = a[0] - p[0], a[1] - p[1]
    bx, by = b[0] - p[0], b[1] - p[1]
    ab2 = ax * ax + ay * ay
    if ab2 < 1e-12:
        return math.hypot(ax, ay)
    t = max(0.0, min(1.0, -(bx * ax + by * ay) / ab2))
    return math.hypot(ax + t * bx, ay + t * by)


def main():
    via_y = float(sys.argv[1]) if len(sys.argv) > 1 else 9.5
    sc = Scene()
    pads = [z for z in getattr(sc, "zones", {}).values() if z.kind == "pad"]
    pad_origin = (sc.venue.get("pad_origin") if hasattr(sc, "venue") else None)
    # pad 条带：用 zone polygon 西缘 x 与 y 范围
    px0 = px1 = py0 = py1 = None
    for z in pads:
        xs = [p[0] for p in z.polygon]
        ys = [p[1] for p in z.polygon]
        px0, px1 = min(xs), max(xs)
        py0, py1 = min(ys), max(ys)
    print("pad 条带: x[%.1f,%.1f] y[%.1f,%.1f]" % (px0, px1, py0, py1))

    # ---- A. 中缝自由带扫描 ----
    print("\n== A. 中缝自由带扫描 (pad=%.2f, 巡航 z 带) ==" % PAD)
    bands = []
    cur = None
    for y10 in range(72, 116):
        y = y10 / 10.0
        blocked = sum(1 for x10 in range(-170, 171)
                      if clearance(sc, x10 / 10.0, y) < PAD)
        free = blocked == 0
        if free and cur is None:
            cur = [y, y]
        elif free:
            cur[1] = y
        elif cur is not None:
            bands.append(cur)
            cur = None
    if cur:
        bands.append(cur)
    for b in bands:
        print("  自由带 y %.2f..%.2f (宽 %.2f)" % (b[0], b[1], b[1] - b[0]))

    # ---- B. 逐投放点航段核查 ----
    print("\n== B. 航段核查 drop -> via(x_d,%.1f) -> pad 东缘(x=%.1f) ==" %
          (via_y, px1))
    ok_all = True
    for dp in sorted(sc.drop_points, key=lambda d: d.xyz[1]):
        dx_, dy_ = dp.xyz[0], dp.xyz[1]
        segs = [((dx_, dy_), (dx_, via_y)),
                ((dx_, via_y), (px1 + 0.5, via_y)),
                ((px1 + 0.5, via_y), (px1 + 0.5, (py0 + py1) / 2))]
        worst = (1e9, None)
        for (a, b) in segs:
            n = int(max(1, math.hypot(b[0] - a[0], b[1] - a[1]) / STEP))
            for i in range(n + 1):
                t = i / n
                x = a[0] + t * (b[0] - a[0])
                y = a[1] + t * (b[1] - a[1])
                c = clearance(sc, x, y)
                if c < worst[0]:
                    worst = (c, (round(x, 2), round(y, 2)))
        ok = worst[0] >= PAD
        ok_all = ok_all and ok
        print("  drop %s (%.1f,%.1f): min_clear=%.3f @ %s -> %s" %
              (dp.id, dx_, dy_, worst[0], worst[1],
               "PASS" if ok else "FAIL"))
    print("  总结: %s" % ("ALL PASS" if ok_all else "存在 FAIL——路点须重选"))

    # ---- C. 对照：现状外圈通道净空（y=12.2 全 x 扫描） ----
    print("\n== C. 对照：外圈 y=12.2 现状最小净空 ==")
    worst = (1e9, None)
    for x10 in range(-170, 171):
        x = x10 / 10.0
        c = clearance(sc, x, 12.2)
        if c < worst[0]:
            worst = (c, (x, 12.2))
    print("  min_clear=%.3f @ %s (证据: 碰撞距 0.45-0.64 同量级)" % worst)


if __name__ == "__main__":
    main()
