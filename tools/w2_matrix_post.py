#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tools/w2_matrix_post.py — W2 矩阵事后归因：碰撞→障碍归属 / 返航时长 / 扣分。

口径（docs/wave2_corridor_case.md 毙杀条件）：
  P1 签名：RETURN 腿 y>11 采样=0、NE+curb 碰撞=0、返航时长 +20s 内；
  P1 毙杀：南绕致 DROP_SIDE/zone 碰撞上升；P2 毙杀：超窗/新增 FAIL。
用法: python3 tools/w2_matrix_post.py <matrix_outbase>
"""
import glob
import math
import os
import re
import sys

import yaml

WS = "/home/ubuntu/zx2026_arena_ws"
sys.path.insert(0, WS + "/src/zx2026_common/scripts")
from zx2026_common.scene import Scene  # noqa: E402

sc = Scene()


def nearest_obstacle(x, y):
    """(kind, label, dist)——2D 最近障碍（树干面/围栏/路缘面）。"""
    best = (None, "?", 1e9)
    for i, ob in enumerate(sc.obstacles):
        if ob.kind == "tree":
            d = math.hypot(x - ob.cx, y - ob.cy) - ob.trunk_r
            lab = "tree#%d(%.1f,%.1f)" % (i, ob.cx, ob.cy)
        else:
            lo, hi = ob.lo, ob.hi
            dx = max(lo[0] - x, 0, x - hi[0])
            dy = max(lo[1] - y, 0, y - hi[1])
            d = math.hypot(dx, dy)
            lab = "%s@(%s)" % (ob.kind, ",".join("%.1f" % v for v in lo[:2]))
        if d < best[2]:
            best = (ob.kind, lab, d)
    return best


def leg_of(x, y):
    """粗腿归属（证据足够对账用，不做精确相位）。"""
    if y > 11.0:
        return "OUTER_RING"
    if 12.5 <= x <= 22.8 and -6 <= y <= 11:
        return "EAST_DROP"
    if x < -16.7:
        return "WEST_PAD"
    if 6.0 <= y <= 9.5 and -13 <= x <= 14:
        return "SEAM_CORRIDOR"
    return "MID"


def ts_of(line):
    m = re.search(r"\[(\d+)\.(\d+)\]", line)
    return float(m.group(1)) + float(m.group(2)) / 1e9 if m else None


def main():
    base = sys.argv[1]
    print("== 碰撞归属（去重：位置+drone） ==")
    for cell in sorted(glob.glob(base + "_*")):
        if cell.endswith("__contaminated") or not os.path.isdir(cell):
            continue
        tag = os.path.basename(cell)
        ev = open(os.path.join(cell, "evidence.txt"),
                  encoding="utf-8", errors="replace").read()
        cols = []
        for ln in ev.splitlines():
            m = re.search(r"drone (\d+) collision marker at "
                          r"\((-?[\d.]+),(-?[\d.]+)\)", ln)
            if not m:
                continue
            key = (m.group(1), round(float(m.group(2)), 1),
                   round(float(m.group(3)), 1))
            if key not in [(c[0], c[1], c[2]) for c in cols]:
                cols.append((key[0], key[1], key[2], ln))
        if not cols:
            print("%-24s col=0" % tag)
            continue
        print("%-24s col=%d" % (tag, len(cols)))
        for (dr, x, y, ln) in cols:
            kind, lab, d = nearest_obstacle(x, y)
            print("    d%s (%6.1f,%6.1f) -> %-7s %s dist=%.2f  leg=%s"
                  % (dr, x, y, kind, lab, d, leg_of(x, y)))

    print("\n== 返航时长（return start -> TOUCHDOWN，秒，逐机） ==")
    for cell in sorted(glob.glob(base + "_*")):
        if cell.endswith("__contaminated") or not os.path.isdir(cell):
            continue
        tag = os.path.basename(cell)
        ev = open(os.path.join(cell, "evidence.txt"),
                  encoding="utf-8", errors="replace").read()
        starts, lands, vias = {}, {}, 0
        for ln in ev.splitlines():
            m = re.search(r"drone (\d+) (?:return start|RETURN-VIA \()", ln)
            if m:
                starts[m.group(1)] = ts_of(ln)
            if "RETURN-VIA" in ln:
                vias += 1
            m = re.search(r"drone (\d+) TOUCHDOWN", ln)
            if m:
                lands[m.group(1)] = ts_of(ln)
        durs = ["d%s:%.0f" % (d, lands[d] - starts[d])
                for d in sorted(starts) if d in lands and starts[d]]
        print("%-24s via_rows=%-3d %s" % (tag, vias, " ".join(durs)))

    print("\n== 扣分明细（score<100 的机） ==")
    for cell in sorted(glob.glob(base + "_*")):
        if cell.endswith("__contaminated") or not os.path.isdir(cell):
            continue
        tag = os.path.basename(cell)
        sy_path = os.path.join(cell, "score.yaml")
        if not os.path.exists(sy_path):
            continue
        try:
            sy = yaml.safe_load(open(sy_path))
        except Exception:
            continue
        tot = sy.get("total", 100)
        if tot >= 100:
            continue
        bad = []
        scores = sy.get("scores", {})
        if isinstance(scores, dict):
            for k, v in scores.items():
                try:
                    s = float(v.get("total", 100)) if isinstance(v, dict) \
                        else float(v)
                except (TypeError, ValueError):
                    continue
                if s < 100:
                    bad.append("d%s=%.0f" % (k, s))
        print("%-24s total=%.0f  %s" % (tag, tot, " ".join(bad) or
                                        "(per-drone 明细缺，查 verify.txt)"))


if __name__ == "__main__":
    main()
